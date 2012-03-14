
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_components/util.h>
#include <bard_components/controllers/gravity_compensation.h>

using namespace bard_components::controllers;

GravityCompensation::GravityCompensation(string const& name) :
  TaskContext(name)
  // Properties
  ,root_link_("")
  ,tip_link_("")
  ,joint_state_throttle_period_(0.01)
  // Operation Callers
  ,get_robot_properties_("get_robot_properties")
  // Robot properties
  ,n_wam_dof_(7)
  ,robot_description_("")
  ,joint_prefix_("")
  // Working variables
  ,kdl_tree_()
  ,kdl_chain_()
  ,id_solver_(NULL)
  ,ext_wrenches_(n_wam_dof_)
  ,positions_(n_wam_dof_)
  ,velocities_(n_wam_dof_)
  ,accelerations_(n_wam_dof_)
  ,torques_(n_wam_dof_)
  ,joint_state_()
  ,joint_state_pub_time_(0)
{
  // Declare properties
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_).doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_).doc("Output port: sensor_msgs/JointState of commanded joint state.");

  // Configure operations
  this->requires("robot_properties")
    ->addOperationCaller(get_robot_properties_);
}

bool GravityCompensation::configureHook()
{
  // Make sure this controller has been connected to a WAM robot
  if(!this->requires("robot_properties")->ready()) {
    std::cerr<<"WARNING: no connection to robot component, needed for robot properties"<<std::endl;
    return false;
  }

  // Get properties from wam robot
  get_robot_properties_(n_wam_dof_, robot_description_, joint_prefix_);

  // Construct an URDF model from the xml string
  urdf::Model urdf_model;
  urdf_model.initString(robot_description_);

  // Get root link
  std::string root_name = urdf_model.getRoot()->name;

  // Get a KDL tree from the robot URDF
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  // Populate the KDL chain
  if(!kdl_tree_.getChain(
        joint_prefix_+"/"+root_link_,
        joint_prefix_+"/"+tip_link_,
        kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: "
        <<joint_prefix_<<"/"<<root_link_
        <<" --> "
        <<joint_prefix_<<"/"<<tip_link_
        <<std::endl
        <<"  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints"
        <<"  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments"
        <<"  The segments are:"
        );

    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin();
        it != segment_map.end();
        it++ )
    {
      ROS_ERROR_STREAM( "    "<<(*it).first);
    }
  
    return false;
  }

  // Create chainsolver
  id_solver_.reset(
      new KDL::ChainIdSolver_RNE(
        kdl_chain_,
        KDL::Vector(0,0,-9.8)));

  // Resize working vectors
  positions_.resize(n_wam_dof_);
  velocities_.resize(n_wam_dof_);
  accelerations_.resize(n_wam_dof_);
  torques_.resize(n_wam_dof_);
  ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

  // Zero out torque data
  torques_.data.setZero();

  // Construct ros JointState message
  util::init_wam_joint_state(
      n_wam_dof_,
      joint_prefix_,
      joint_state_);

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);
  joint_state_out_port_.setDataSample(joint_state_);

  return true;
}

bool GravityCompensation::startHook()
{
  return true;
}

void GravityCompensation::updateHook()
{
  // Read in the current joint positions
  positions_in_port_.read( positions_ );

  // Compute inverse dynamics
  // This computes the torques on each joint of the arm as a function of
  // the arm's joint-space position, velocities, accelerations, external
  // forces/torques and gravity.
  if(id_solver_->CartToJnt(
        positions_,
        velocities_,
        accelerations_,
        ext_wrenches_,
        torques_) != 0)
  {
    std::cerr<<"ERROR: Could not compute joint torques!"<<std::endl;
  }
 
  // Send joint positions
  torques_out_port_.write( torques_ );
  
  // Copy the command into a sensor_msgs/JointState message
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

void GravityCompensation::stopHook()
{
}

void GravityCompensation::cleanupHook()
{
}
