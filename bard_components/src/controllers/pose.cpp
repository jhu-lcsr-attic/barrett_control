#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <tf_conversions/tf_kdl.h>

#include <bard_components/util.h>
#include <bard_components/controllers/pose.h>

using namespace bard_components::controllers;

CartesianPose::CartesianPose(string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame_("")
  ,kp_(7,0.0)
  ,kd_(7,0.0)
  ,joint_state_throttle_period_(0.01)
  // Working variables
  ,n_dof_(0)
  ,kdl_chain_()
  ,kdl_tree_()
  ,positions_()
  ,torques_()
  ,joint_state_()
  ,joint_state_throttle_(joint_state_throttle_period_)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  this->addProperty("kd",kd_);
  this->addProperty("kp",kp_);

  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("target_frame",target_frame_)
    .doc("The target frame to track with tip_link.");
  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_)
     .doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure data ports
  this->ports()->addEventPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("positions_out", positions_out_port_)
    .doc("Output port: nx1 vector of desired joint positins. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_)
   .doc("Output port: sensor_msgs::JointState.");

  this->addOperation("testIK", &CartesianPose::test_ik, this, RTT::OwnThread)
    .doc("Test the IK computation.");
}

bool CartesianPose::configureHook()
{
  // Connect to tf
  if(this->hasPeer("tf")) {
    TaskContext* tf_task = this->getPeer("tf");
    tf_lookup_transform_ = tf_task->getOperation("lookupTransform"); // void reset(void)
  } else {
    ROS_ERROR("CartesianPose controller is not connected to tf!");
    return false;
  }

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  if(!bard_components::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }
  
  // Make sure we have enough gains
  if(kp_.size() < n_dof_ || kd_.size() < n_dof_) {
    ROS_ERROR("Not enough gains!");
    return false;
  }

  
  // Resize working variables
  positions_.resize(n_dof_);
  positions_des_.resize(n_dof_);
  joint_limits_min_.resize(n_dof_);
  joint_limits_max_.resize(n_dof_);
  torques_.resize(n_dof_);

  // Get joint limits from URDF model
  {
    unsigned int i=0;
    for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
        it != kdl_chain_.segments.end();
        it++)
    {
      joint_limits_min_(i) = urdf_model_.joints_[it->getJoint().getName()]->limits->lower;
      joint_limits_max_(i) = urdf_model_.joints_[it->getJoint().getName()]->limits->upper;
      i++;
    }
  }

  // Initialize IK solver
  kdl_fk_solver_pos_.reset(
      new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  kdl_ik_solver_vel_.reset(
      new KDL::ChainIkSolverVel_pinv(
        kdl_chain_,
        1.0E-6,
        150));
  kdl_ik_solver_top_.reset(
      new KDL::ChainIkSolverPos_NR(
        kdl_chain_,
        //joint_limits_min_,
        //joint_limits_max_,
        *kdl_fk_solver_pos_,
        *kdl_ik_solver_vel_,
        150,
        1.0E-6));

  // Zero out torque data
  torques_.data.setZero();
  positions_.q.data.setZero();
  positions_des_.q.data.setZero();
  
  // Construct ros JointState message with the appropriate joint names
  bard_components::util::joint_state_from_kdl_chain(kdl_chain_, joint_state_);

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_des_);
  torques_out_port_.setDataSample(torques_);
  joint_state_out_port_.setDataSample(joint_state_);

  return true;
}

void CartesianPose::test_ik() { this->compute_ik(true); }

void CartesianPose::compute_ik(bool debug)
{
  // Read in the current joint positions
  positions_in_port_.readNewest( positions_ );

  // Get transform from the root link frame to the target frame
  try{
    tip_frame_msg_ = tf_lookup_transform_("/"+root_link_,target_frame_);
  } catch (std::exception &ex) {
    ROS_ERROR_STREAM("Could not look up transform from \""<<root_link_<<"\" to \""<<target_frame_<<"\": "<<ex.what());
    this->stop();
  }
  tf::transformMsgToTF(tip_frame_msg_.transform, tip_frame_tf_);
  tf::TransformTFToKDL(tip_frame_tf_,tip_frame_des_);

  // Compute joint coordinates of the target tip frame
  KDL::JntArray ik_hint(n_dof_);
  ik_hint.data = positions_.q.data;

  kdl_ik_solver_top_->CartToJnt(ik_hint, tip_frame_des_, positions_des_.q);

  // Unwrap angles
  for(unsigned int i=0; i<n_dof_; i++) {
    positions_des_.q(i) = fmod(positions_des_.q(i)+M_PI,2.0*M_PI)-M_PI;
  }

  // Servo in jointspace to the appropriate joint coordinates
  for(unsigned int i=0; i<n_dof_; i++) {
    torques_(i) =
      kp_[i]*(positions_des_.q(i) - positions_.q(i))
      + kd_[i]*(positions_des_.qdot(i) - positions_.qdot(i));
  }

  if(debug) {
    ROS_INFO_STREAM("Current position: "<<positions_.q.data.transpose());
    ROS_INFO_STREAM("Current velocity: "<<positions_.qdot.data.transpose());
    ROS_INFO_STREAM("IK result:        "<<positions_des_.q.data.transpose());
    ROS_INFO_STREAM("Displacement:     "<<(positions_des_.q.data-positions_.q.data).transpose());
    ROS_INFO_STREAM("Torques:          "<<torques_.data.transpose());
  }
  
  // Copy desired joint positions into joint state
  if( joint_state_throttle_.ready(joint_state_throttle_period_)) {
    joint_state_.header.stamp = ros::Time::now();
    for(unsigned int i=0; i<n_dof_; i++) {
      joint_state_.position[i] = positions_des_.q(i);
      joint_state_.effort[i] = torques_(i);
    }
    joint_state_out_port_.write( joint_state_ );
  } 
}

bool CartesianPose::startHook()
{
  try{
    tip_frame_msg_ = tf_lookup_transform_(root_link_,target_frame_);
  } catch (std::exception &ex) {
    ROS_ERROR_STREAM("Could not look up transform from \""<<root_link_<<"\" to \""<<target_frame_<<"\": "<<ex.what());
    return false;
  }
  return true;
}

void CartesianPose::updateHook()
{
  // Compute the inverse kinematics solution
  this->compute_ik(false);

  // Send position target
  positions_out_port_.write( positions_des_ );
  
  // Send joint torques 
  torques_out_port_.write( torques_ );
}

void CartesianPose::stopHook()
{
}

void CartesianPose::cleanupHook()
{
}


