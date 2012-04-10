#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <tf_conversions/tf_kdl.h>

#include <bard_controllers/util.h>
#include <bard_controllers/controllers/wrench.h>

using namespace bard_controllers::controllers;

CartesianWrench::CartesianWrench(string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame_("")
  ,kp_(6,0.0)
  ,kd_(6,0.0)
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
  ,positions_()
  ,torques_()
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

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
}

bool CartesianWrench::configureHook()
{
  // Connect to tf
  if(this->hasPeer("tf")) {
    TaskContext* tf_task = this->getPeer("tf");
    tf_lookup_transform_ = tf_task->getOperation("lookupTransform"); // void reset(void)
  } else {
    ROS_ERROR("CartesianWrench controller is not connected to tf!");
    return false;
  }

  // Make sure we have enough gains
  if(kp_.size() < 6 || kd_.size() < 6) {
    ROS_ERROR("Not enough gains!");
    return false;
  }

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!bard_common::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }

  // Initialize cartesian position solver
  kdl_fk_solver_pos_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  kdl_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize working variables
  positions_.resize(n_dof_);
  torques_.resize(n_dof_);
  jacobian_.resize(n_dof_);

  // Zero out torque data
  torques_.data.setZero();

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);

  return true;
}

bool CartesianWrench::startHook()
{
  try{
    tip_frame_msg_ = tf_lookup_transform_(root_link_,target_frame_);
  } catch (std::exception &ex) {
    ROS_ERROR_STREAM("Could not look up transform from \""<<root_link_<<"\" to \""<<target_frame_<<"\": "<<ex.what());
    return false;
  }
  return true;
}

void CartesianWrench::updateHook()
{

  // Read in the current joint positions
  positions_in_port_.read( positions_ );

  // Compute the forward kinematics and Jacobian (at this location).                                                                                                              
  kdl_fk_solver_pos_->JntToCart(positions_.q, tip_frame_);
  kdl_jacobian_solver_->JntToJac(positions_.q, jacobian_);

  // Compute cartesia velocity
  // FIXME: this is really gross... seriously, KDL?
  // TODO: jacobian_.data is an Eigen dynamic matrix, so we should be able to just multiply these things out
  // TODO: fix all the matrix arithmetic here so they drop down to eigen types
  for (unsigned int i = 0 ; i < 6 ; i++) {
    cart_vel_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++) {
      cart_vel_(i) += jacobian_(i,j) * positions_.qdot(j);
    }
  }

  // Get transform from the root link frame to the target frame
  try{
    tip_frame_msg_ = tf_lookup_transform_("/"+root_link_,target_frame_);
  } catch (std::exception &ex) {
    ROS_ERROR_STREAM("Could not look up transform from \""<<root_link_<<"\" to \""<<target_frame_<<"\": "<<ex.what());
    this->stop();
  }
  tf::transformMsgToTF(tip_frame_msg_.transform,tip_frame_tf_);
  tf::TransformTFToKDL(tip_frame_tf_,tip_frame_des_);
  
  // Construct twist for position error
  cart_twist_err_.vel = tip_frame_des_.p - tip_frame_.p;
  cart_twist_err_.rot = -0.5 * (
      tip_frame_des_.M.UnitX() * tip_frame_.M.UnitX() +
      tip_frame_des_.M.UnitY() * tip_frame_.M.UnitY() +
      tip_frame_des_.M.UnitZ() * tip_frame_.M.UnitZ());

  // TODO: Construct twist for velocity error

  // Apply gains to position and velocity error
  for (unsigned int i = 0 ; i < 6 ; i++) {
    cart_effort_(i) =  kp_[i] * cart_twist_err_(i) + kd_[i] * (0.0 - cart_vel_(i));
  }

  // Convert the force into a set of joint torques.                                                                                                                               
  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) {
    torques_(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++) {
      torques_(i) += jacobian_(j,i) * cart_effort_(j);
    }
  }

  // Send joint torques 
  torques_out_port_.write( torques_ );
}

void CartesianWrench::stopHook()
{
}

void CartesianWrench::cleanupHook()
{
}


