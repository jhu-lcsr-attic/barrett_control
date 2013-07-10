#include <ros/ros.h>
#include <native/task.h>
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <controller_manager/controller_manager.h>
#include <signal.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Duration.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <libconfig.h++>

#include <barrett/exception.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/bus/can_socket.h>
#include <barrett/products/product_manager.h>

#include <barrett_model/semi_absolute_joint_interface.h>

#include <terse_roscpp/param.h>

#include <urdf/model.h>

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

namespace barrett_hw 
{ 
  class BarrettHW : public hardware_interface::RobotHW 
  {
  public:
    BarrettHW(ros::NodeHandle nh);
    bool configure();
    bool start();
    bool read(const ros::Time time, const ros::Duration period); 
    void write(const ros::Time time, const ros::Duration period);
    void stop();
    void cleanup();

    bool wait_for_active();

    //std::map<std::string, boost::shared_ptr<barrett::MultiPuckProduct> > products;
    typedef std::map<std::string, boost::shared_ptr<barrett::ProductManager> >  ManagerMap;
    typedef std::map<std::string, boost::shared_ptr<barrett::LowLevelWam<4> > > Wam4Map;
    typedef std::map<std::string, boost::shared_ptr<barrett::LowLevelWam<7> > > Wam7Map;
    typedef std::map<std::string, boost::shared_ptr<barrett::Hand > > HandMap;

  private:
    ros::NodeHandle nh_;
    size_t n_total_dof_;

    // ros-controls interface
    std::vector<std::string> joint_names_;
    Eigen::VectorXd joint_positions_, joint_velocities_;
    Eigen::VectorXd joint_effort_cmds_;
    Eigen::VectorXd resolver_angles_, resolver_ranges_, joint_offsets_;
    Eigen::VectorXi calibrated_joints_;

    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::EffortJointInterface effort_interface_;
    barrett_model::SemiAbsoluteJointInterface semi_absolute_interface_;
    
    // Vectors for various barrett products
    ManagerMap barrett_managers_;
    Wam4Map wam4s_;
    Wam7Map wam7s_;
    HandMap hands_;

    template<int DOF>
    Eigen::Matrix<double,DOF,1> compute_resolver_ranges(boost::shared_ptr<barrett::LowLevelWam<DOF> > wam) 
    {
      Eigen::MatrixXd m_to_j_pos = wam->getMotorToJointPositionTransform();  
      return (m_to_j_pos.diagonal().array() * 2.0*M_PI).cwiseAbs().matrix();
    }

  };

  BarrettHW::BarrettHW(ros::NodeHandle nh) :
    nh_(nh),
    n_total_dof_(0)
  {

  }

  bool BarrettHW::configure() 
  {
    using namespace terse_roscpp;
    std::vector<std::string> product_names;

    // Get URDF
    std::string urdf_str;
    param::require(nh_, "robot_description", urdf_str, "The URDF for this barrett system.");
    urdf::Model urdf;
    urdf.initString(urdf_str);

    // Load parameters
    param::require(nh_,"product_names",product_names, "The unique barrett product names.");

    for(std::vector<std::string>::const_iterator it = product_names.begin();
        it != product_names.end();
        ++it) 
    {
      const std::string &product_name = *it;
      ros::NodeHandle product_nh(nh_,"products/"+product_name);

      // Determine the bus for this product
      std::string bus_name;
      param::require(product_nh,"bus",bus_name, "Bus name.");

      // Create the product manager if it doesn't exist
      boost::shared_ptr<barrett::ProductManager> barrett_manager;

      if(barrett_managers_.find(bus_name) == barrett_managers_.end()) {
        // Determine the bus information 
        int bus_port;
        param::require(product_nh,"busses/"+bus_name+"/port", bus_port, "Bus port [0-n].");
        std::string config_path;
        bool config_path_found = param::get(product_nh,"busses/"+bus_name+"/config", config_path, "Path to a libbarrett config file.");

        // Create a new bus/manager
        boost::shared_ptr<barrett::bus::CANSocket> canbus(new barrett::bus::CANSocket(bus_port));
        barrett_manager.reset(
            new barrett::ProductManager(
              config_path_found ? config_path.c_str() : NULL /* Use defailt config */,
              canbus.get()));
        barrett_managers_[bus_name] = barrett_manager;
      } else {
        // Use the existing bus/manager
        barrett_manager = barrett_managers_[bus_name];
      }

      // Get the product information
      std::string product_type;
      param::require(product_nh,"type",product_type, "Barrett product type [wam,bhand].");

      // Add a WAM
      if(product_type == "wam") {
        // Get the configuration for this type of arm
        const libconfig::Setting& wam_config = barrett_manager->getConfig().lookup(barrett_manager->getWamDefaultConfigPath());

        // Determine the DOF of this wam
        size_t dof = 0;

        if(barrett_manager->foundWam4()) { dof = 4; }
        else if(barrett_manager->foundWam7()) { dof = 7; }
        else { ROS_ERROR("Could not find WAM on bus!"); continue; }

        // Resize state vectors
        n_total_dof_ += dof;
        joint_positions_.resize(n_total_dof_);
        joint_velocities_.resize(n_total_dof_);
        joint_effort_cmds_.resize(n_total_dof_);
        calibrated_joints_.resize(n_total_dof_);
        joint_offsets_.resize(n_total_dof_);
        resolver_angles_.resize(n_total_dof_);
        resolver_ranges_.conservativeResize(n_total_dof_);

        // Construct the wam interface
        Eigen::VectorXd resolver_ranges;

        // Get the pucks from this wam
        std::vector<barrett::Puck*> wam_pucks = barrett_manager->getWamPucks();
        wam_pucks.resize(dof);

        // Store the wam interface
        switch(dof) {
          case 4: { boost::shared_ptr<barrett::LowLevelWam<4> > wam_ptr(new barrett::LowLevelWam<4>(wam_pucks, barrett_manager->getSafetyModule(), wam_config["low_level"]));
                    resolver_ranges_.bottomRows<4>() = this->compute_resolver_ranges<4>(wam_ptr);
                    wam4s_[product_name] = wam_ptr; }
                  break;
          case 7: { boost::shared_ptr<barrett::LowLevelWam<7> > wam_ptr(new barrett::LowLevelWam<7>(wam_pucks, barrett_manager->getSafetyModule(), wam_config["low_level"]));
                    resolver_ranges_.bottomRows<7>() = this->compute_resolver_ranges<7>(wam_ptr);
                    wam7s_[product_name] = wam_ptr; }
                  break;
        };

        // Get URDF links starting at product root link
        std::string tip_joint_name;
        param::require(product_nh,"tip_joint",tip_joint_name, "WAM tip joint name in URDF.");
        boost::shared_ptr<const urdf::Joint> joint = urdf.getJoint(tip_joint_name);

        // Create joint handles
        for(size_t i=0; i<dof; i++) {
          // While the joint has been handled or the joint type isn't revolute
          while(std::find(joint_names_.begin(),joint_names_.end(),joint->name) != joint_names_.end() 
                || joint->type != urdf::Joint::REVOLUTE)
          {
            // Get the next joint
            joint = urdf.getLink(joint->parent_link_name)->parent_joint;
            // Make sure we didn't run out of links
            if(!joint.get()) {
              ROS_ERROR_STREAM("Ran out of joints while parsing URDF starting at joint: "<<tip_joint_name);
              // TODO: throw exception, clean up everything
              return false;
            }
          }
          
          // Store the joint name
          joint_names_.push_back(joint->name);

          // Joint State Handle
          hardware_interface::JointStateHandle state_handle(joint->name,
              &joint_positions_(i),
              &joint_velocities_(i),
              &joint_effort_cmds_(i));
          state_interface_.registerHandle(state_handle);

          // Effort Command Handle
          effort_interface_.registerHandle(
              hardware_interface::JointHandle(
                state_interface_.getHandle(joint->name),
                &joint_effort_cmds_(i)));

          // Transmission / Calibration handle
          semi_absolute_interface_.registerJoint(
              effort_interface_.getHandle(joint->name),
              resolver_ranges_(i),
              &resolver_angles_(i),
              &joint_offsets_(i),
              &calibrated_joints_(i));

        }


      } else if(product_type == "hand") {
        ROS_ERROR_STREAM("Look ma, no hands!");
        continue;
      } else {
        ROS_ERROR_STREAM("Unknown Barrett product type: "+product_type);
        continue;
      }
    }

    // TODO: Set up gravity compensator
    // Write kdl_ros_integration package which handles kdl/urdf/ros_control? interfaces

    return true;
  }

  bool BarrettHW::wait_for_active() 
  {
    for(ManagerMap::iterator it = barrett_managers_.begin(); 
        it != barrett_managers_.end();
        ++it) 
    {
      it->second->getSafetyModule()->waitForMode(barrett::SafetyModule::ACTIVE);
    }
    return true;
  }
}

int main( int argc, char** argv ){

  // Set up real-time task
  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupWAM", 99, 0 );

  // Initialize ROS
  ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Construct the wam structure
  ros::NodeHandle barrett_nh("barrett");

  barrett_hw::BarrettHW barrett(barrett_nh);


  //TODO: execution manager not needed for LLW
  //barrett_manager.getExecutionManager()->start();

  //LowLevelWam::getJointPositions()
  //LowLevelWam::setTorques()
  //LowLevelWam::update()

  ////barrett_hw::WAM wam_hw( barrett_nh );

  // Timer variables
#if 0 
  struct timespec ts = {0,0};

  if(clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    ROS_FATAL("Failed to poll realtime clock!");
  }

  ros::Time 
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(barrett_nh, "loop_rate", 2);

  bool wam_ok = false;
  while(!g_quit && !wam_ok) {
    if(!wam_hw.configure()) {
      ROS_ERROR("Could not configure WAM!");
    } else if(!wam_hw.start()) {
      ROS_ERROR("Could not start WAM!");
    } else {
      ros::Duration(1.0).sleep();

      if(!wam_hw.read(now, period)) {
        ROS_ERROR("Could not read from WAM!");
      } else {
        wam_ok = true;
      }
    }

    ros::Duration(1.0).sleep();
  }

  // Construct the controller manager
  ros::NodeHandle nh;
  controller_manager::ControllerManager manager(&wam_hw, nh);

  uint32_t count = 0;

  // Run as fast as possible
  while( !g_quit ) {
    // Get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    } 

    // Read the state from the WAM
    if(!wam_hw.read(now, period)) {
      g_quit=true;
      break;
    }

    // Update the controllers
    manager.update(now, period);

    // Write the command to the WAM
    wam_hw.write(now, period);

    if(count++ > 1000) {
      if(publisher.trylock()) {
        count = 0;
        publisher.msg_.data = period;
        publisher.unlockAndPublish();
      }
    }
  }

  publisher.stop();

  std::cerr<<"Stpping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping WAM..."<<std::endl;
  wam_hw.stop();

  std::cerr<<"Cleaning up WAM..."<<std::endl;
  wam_hw.cleanup();
#endif

  std::cerr<<"Poka!"<<std::endl;

  return 0;
}

