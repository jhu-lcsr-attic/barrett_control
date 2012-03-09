
#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <bard_components/util.h>
#include <bard_components/wam.h>

using namespace bard_components;

WAM::WAM(string const& name) :
  TaskContext(name, RTT::base::TaskCore::PreOperational)
  // Properties
  ,n_wam_dof_(0)
  ,can_dev_name_("")
  ,robot_description_("")
  ,joint_prefix_("")
  ,initial_positions_(7,0.0)
  ,joint_state_throttle_period_(0.01)
  // Internal variables
  ,canbus_(NULL)
  ,robot_(NULL)
  ,needs_calibration_(true)
  ,torques_()
  ,positions_()
  ,joint_state_()
  ,joint_state_pub_time_(0)
{
  // Declare properties (configuration variables)
  this->addProperty("n_wam_dof",n_wam_dof_).doc("The number of degrees-of-freedom of the WAM robot (4 or 7).");
  this->addProperty("can_dev_name",can_dev_name_).doc("The name of the RTCAN device to which this WAM robot is connected.");
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("joint_prefix",joint_prefix_).doc("The joint name prefix used in the WAM URDF.");
  this->addProperty("initial_positions",initial_positions_).doc("The calibration position of the robot.");
  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_).doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure data ports
  this->ports()->addEventPort("torques_in", torques_in_port_).doc("Input Event port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("positions_out", positions_out_port_).doc("Output port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_).doc("Output port: sensor_msgs::JointState.");

  // Add operation for getting robot properties
  this->provides("robot_properties")
    ->addOperation("get_robot_properties", &WAM::get_robot_properties, this, RTT::OwnThread);

  // Add operation for setting the encoder values
  this->provides("calibration")
    ->addOperation("calibrate_position", &WAM::calibrate_position, this, RTT::OwnThread)
    .doc("Set the angles that the encoders should read with the arm in the current configuration. This is used for calibrating the robot.")
    .arg("angles","The new joint angles.");

  // Add operations for setting warnings and faults
  this->addOperation("setVelocityWarning", &barrett_direct::WAM::SetVelocityWarning, robot_, RTT::OwnThread)
    .doc("Set the velocities above which the WAM pendant will illumiate a warning light.")
    .arg("thresh","Velocity Warning Threshold");
  this->addOperation("setVelocityFault", &barrett_direct::WAM::SetVelocityFault, robot_, RTT::OwnThread)
    .doc("Set the velocities above which the WAM pendant will abruptly shut down the arm and illumiate a fault light.")
    .arg("thresh","Velocity Fault Threshold");
  this->addOperation("setTorqueWarning", &barrett_direct::WAM::SetTorqueWarning, robot_, RTT::OwnThread)
    .doc("Set the velocities above which the WAM pendant will illumiate a warning light.")
    .arg("thresh","Torque Warning Threshold");
  this->addOperation("setTorqueFault", &barrett_direct::WAM::SetTorqueFault, robot_, RTT::OwnThread)
    .doc("Set the velocities above which the WAM pendant will abruptly shut down the arm and illumiate a fault light.")
    .arg("thresh","Torque Fault Threshold");

  std::cout << "WAM \""<<name<<"\" constructed !" <<std::endl;
}

bool WAM::configureHook()
{
  // TODO: load URDF from  rosparam
  // TODO: get base link of THIS arm from rosparam

  // TODO: get #DOF from URDF
  // TODO: get joint names from URDF

  // Resize joint arrays
  torques_ = KDL::JntArray(n_wam_dof_);
  positions_ = KDL::JntArray(n_wam_dof_);

  // Zero out torques and positions
  torques_.data.setZero();
  positions_.data.setZero();

  // Construct ros JointState message
  util::init_wam_joint_state(n_wam_dof_, prefix_, joint_state_);

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_);
  joint_state_out_port_.setDataSample(joint_state_);

  // Try to connect and initialize hardware
  try{
    // Construct CAN structure
    canbus_.reset(new leoCAN::RTSocketCAN(can_dev_name_, leoCAN::CANBus::RATE_1000 ));

    // Open the canbus
    if( canbus_->Open() != leoCAN::CANBus::ESUCCESS ){
      std::cerr<<"Failed to open CAN device \""<<can_dev_name_<<"\""<<std::endl;
      throw std::exception();
    }

    // Construct WAM structure
    robot_.reset(new barrett_direct::WAM(canbus_, (barrett_direct::WAM::Configuration)n_wam_dof_));

    // Initialize the WAM robot
    if( robot_->Initialize() != barrett_direct::WAM::ESUCCESS ){
      std::cerr<<"Failed to initialize WAM"<<std::endl;
      throw std::exception();
    }
  } catch(std::exception &ex) {
    // Free the device handles
    this->cleanup_internal();
    return false;
  }

  std::cout << "WAM connected on CAN device \""<<can_dev_name_<<"\"!" <<std::endl;

  return true;
}

bool WAM::startHook()
{
  // Check the data ports
  if ( !torques_in_port_.connected() ) {
    std::cerr<<"WARNING: No connection to \"torques_in\" for WAM on \""<<can_dev_name_<<"\"!"<<std::endl;
  }

  if ( !positions_out_port_.connected() ) {
    std::cerr<<"WARNING: No connection to \"positions_out\" for WAM on \""<<can_dev_name_<<"\"!"<<std::endl;
  }

  if(needs_calibration_) {
    // Set the joints to the calibration position
    this->calibrate_position(initial_positions_);
    needs_calibration_ = false;
  }

  // Set the robot to Activated
  if( robot_->SetMode(barrett_direct::WAM::MODE_ACTIVATED) != barrett_direct::WAM::ESUCCESS ){
    std::cerr<<"Failed to ACTIVATE WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
  }

  std::cout << "WAM started on CAN device \""<<can_dev_name_<<"\"!" <<std::endl;
  return true;
}

void WAM::updateHook()
{
  // Get joint positions
  if( robot_->GetPositions( positions_.data ) != barrett_direct::WAM::ESUCCESS) {
    std::cerr<<"Failed to get positions of WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
  }

  // Only send joint torques if new data is coming in
  if( torques_in_port_.read( torques_ ) == RTT::NewData ) {
    if( robot_->SetTorques( torques_.data ) != barrett_direct::WAM::ESUCCESS ) {
      std::cerr<<"Failed to set torques of WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
    }
  }

  // Send joint positions
  positions_out_port_.write( positions_ );

  // Copy joint positions into joint state
  if( RTT::os::TimeService::Instance()->secondsSince(joint_state_pub_time_) > joint_state_throttle_period_ ) {
    joint_state_.header.stamp = ros::Time::now();
    for(int i=0; i<n_wam_dof_; i++) {
      joint_state_.position[i] = positions_(i);
      joint_state_.effort[i] = torques_(i);
    }
    joint_state_out_port_.write( joint_state_ );
    joint_state_pub_time_ = RTT::os::TimeService::Instance()->getTicks();
  } 
}

void WAM::stopHook()
{
  // Set the robot to IDLE
  if( robot_->SetMode(barrett_direct::WAM::MODE_IDLE) != barrett_direct::WAM::ESUCCESS ){
    std::cerr<<"Failed to IDLE WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
  }
}

void WAM::cleanupHook()
{
  // Close the CANBus
  if( canbus_->Close() != leoCAN::CANBus::ESUCCESS ){
    std::cerr<<"Failed to close CAN device \""<<can_dev_name_<<"\""<<std::endl;
  }

  // Reset calibration flag
  needs_calibration_ = true;

  // Free the device handles
  this->cleanup_internal();
}

void WAM::get_robot_properties(
    int &n_wam_dof,
    std::string &robot_description,
    std::string &joint_prefix) 
{
  n_wam_dof = n_wam_dof_;
  robot_description = robot_description_;
  joint_prefix = joint_prefix_;
}

void WAM::calibrate_position(std::vector<double> &actual_positions)
{
  // Make sure we have a connection to the robot
  if(this->isConfigured()) {
    // Assign the positions to the current robot configuration
    if(robot_->SetPositions(Eigen::Map<Eigen::VectorXd>(&actual_positions[0],actual_positions.size())) != barrett_direct::WAM::ESUCCESS)
    {
      std::cerr<<"Failed to calibrate encoders!"<<std::endl;
    }

    std::cerr<<"Calibrated encoders."<<std::endl;
  } else {
    std::cerr<<"Cannot calibrate encoders! The WAM control task on device "<<can_dev_name_<<" is not configured."<<std::endl;
  }
}

void WAM::cleanup_internal()
{
  // Reset the scoped pointers
  robot_.reset(NULL);
  canbus_.reset(NULL);
}
