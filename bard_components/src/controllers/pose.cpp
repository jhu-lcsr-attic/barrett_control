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
  ,Kp_(7,0.0)
  ,Kd_(7,0.0)
  // Working variables
  ,n_dof_(0)
  ,kdl_chain_()
  ,kdl_tree_()
  ,positions_()
  ,torques_()
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  this->addProperty("Kd",Kd_);
  this->addProperty("Kp",Kp_);

  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("target_frame",target_frame_)
    .doc("The target frame to track with tip_link.");

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
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

  // Make sure we have enough gains
  if(Kp_.size() < 6 || Kd_.size() < 6) {
    ROS_ERROR("Not enough gains!");
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
  
  // Resize working variables
  positions_.resize(n_dof_);
  positions_des_.resize(n_dof_);
  joint_limits_min_.resize(n_dof_);
  joint_limits_max_.resize(n_dof_);
  torques_.resize(n_dof_);

  // Get joint limits from URDF model
  {
    unsigned int i=0;
    for(std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it=urdf_model_.joints_.begin();
        it != urdf_model_.joints_.end();
        it++)
    {
      joint_limits_min_(i) = it->second->limits->lower;
      joint_limits_max_(i) = it->second->limits->upper;
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
      new KDL::ChainIkSolverPos_NR_JL(
        kdl_chain_,
        joint_limits_min_,
        joint_limits_max_,
        *kdl_fk_solver_pos_,
        *kdl_ik_solver_vel_,
        100,
        1.0E-6));

  // Zero out torque data
  torques_.data.setZero();

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);

  return true;
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
  // Read in the current joint positions
  positions_in_port_.read( positions_ );

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
  kdl_ik_solver_top_->CartToJnt(positions_.q, tip_frame_des_, positions_des_.q);

  // Servo in jointspace to the appropriate joint coordinates
  //torques_.data = 
  //  Eigen::Map<Eigen::VectorXd>(&Kp_[0],Kp_.size()).cwiseProduct(positions_des_.q.data - positions_.q.data)
  //  + Eigen::Map<Eigen::VectorXd>(&Kd_[0],Kd_.size()).cwiseProduct(positions_des_.qdot.data - positions_.qdot.data);

  for(unsigned int i=0; i<n_dof_; i++) {
    torques_(i) =
      Kp_[i]*(positions_des_.q(i) - positions_.q(i))
      + Kd_[i]*(positions_des_.qdot(i) - positions_.qdot(i));
  }
  
  // Send joint torques 
  torques_out_port_.write( torques_ );
}

void CartesianPose::stopHook()
{
}

void CartesianPose::cleanupHook()
{
}


