#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_components/util.h>
#include <bard_components/controllers/joint_pid.h>

using namespace bard_components::controllers;

JointPID::JointPID(string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,kp_(7,0.0)
  ,ki_(7,0.0)
  ,i_clamp_(7,0.0)
  ,kd_(7,0.0)
  ,joint_state_throttle_period_(0.01)
  // Working variables
  ,n_dof_(0)
  ,kdl_chain_()
  ,kdl_tree_()
  ,positions_()
  ,positions_des_()
  ,torques_()
  ,joint_state_()
  ,joint_state_throttle_(joint_state_throttle_period_)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("kp",kp_).doc("The joint proportional gains.");
  this->addProperty("ki",ki_).doc("The joint integral gains.");
  this->addProperty("i_clamp",i_clamp_).doc("The joint integral error clamp. This limits integral wind-up.");
  this->addProperty("kd",kd_).doc("The joint derivative gains.");

  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");

  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_)
     .doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure data ports
  this->ports()->addEventPort("positions_in", positions_in_port_).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("positions_des_in", positions_des_in_port_).doc("Input port: nx1 vector of desired joint positions. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_)
   .doc("Output port: sensor_msgs::JointState.");
}

bool JointPID::configureHook()
{
  // Construct an URDF model from the xml string
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  if(!bard_components::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }

  // Check gains
  if(kp_.size() < n_dof_ || kd_.size() < n_dof_) {
    ROS_ERROR("Not enough gains for controller.");
    return false;
  }

  // Resize working vectors
  positions_.resize(n_dof_);
  positions_des_.resize(n_dof_);
  positions_des_.q.data.setZero();
  positions_des_.qdot.data.setZero();

  p_error_.resize(n_dof_);
  i_error_.resize(n_dof_);
  d_error_.resize(n_dof_);
  torques_.resize(n_dof_);

  // Zero out torque data
  p_error_.data.setZero();
  i_error_.data.setZero();
  d_error_.data.setZero();
  torques_.data.setZero();
  
  // Construct ros JointState message with the appropriate joint names
  bard_components::util::joint_state_from_kdl_chain(kdl_chain_, joint_state_);

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);
  joint_state_out_port_.setDataSample(joint_state_);

  return true;
}

bool JointPID::startHook()
{
  positions_des_.q.data.setZero();
  positions_des_.qdot.data.setZero();
  return true;
}

void JointPID::updateHook()
{
  // Read in the current joint positions & velocities
  positions_in_port_.readNewest( positions_ );

  // Read in the goal joint positions & velocities
  if(positions_des_in_port_.readNewest( positions_des_ ) ) {

    // Compute torques
    for(unsigned int i=0; i<n_dof_; i++) {
      // Compute proportional and derivative error
      p_error_(i) = positions_des_.q(i) - positions_.q(i);
      d_error_(i) = positions_des_.qdot(i) - positions_.qdot(i);
      // Integrate proportional error if it is below the integral clamp
      if(fabs(i_error_(i)) < fabs(i_clamp_[i])) {
        i_error_(i) += p_error_(i);
      }
      // Compute pid joint torque
      torques_(i) = kp_[i]*p_error_(i) + ki_[i]*i_error_(i) + kd_[i]*d_error_(i) ;
    }
  } else {
    torques_.data.setZero();
  }
 
  // Send joint torques
  torques_out_port_.write( torques_ );

  
  // Copy desired joint positions into joint state
  if( joint_state_throttle_.ready(joint_state_throttle_period_)) {
    joint_state_.header.stamp = ros::Time::now();
    for(unsigned int i=0; i<n_dof_; i++) {
      joint_state_.position[i] = positions_des_.q(i);
      joint_state_.velocity[i] = positions_des_.qdot(i);
      joint_state_.effort[i] = torques_(i);
    }
    joint_state_out_port_.write( joint_state_ );
  } 
}

void JointPID::stopHook()
{
}

void JointPID::cleanupHook()
{
}

