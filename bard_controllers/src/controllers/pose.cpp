/*
 * Copyright (c) 2012, The Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Johns Hopkins University. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

#include <bard_common/util.h>
#include <bard_controllers/controllers/pose.h>

using namespace bard_controllers::controllers;

CartesianPose::CartesianPose(string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame_("")
  ,kp_(7,0.0)
  ,kd_(7,0.0)
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
  this->ports()->addPort("positions_out", positions_out_port_)
    .doc("Output port: nx1 vector of desired joint positins. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("trajectories_out", trajectories_out_port_)
    .doc("Output port: nx1 vector of joint trajectories. (n joints)");

  this->addOperation("testIK", &CartesianPose::test_ik, this, RTT::OwnThread)
    .doc("Test the IK computation.");
  
  // Initialize properties from rosparam
  bard_common::util::load_rosparam_and_refresh(this);
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
  if(!bard_common::util::initialize_kinematics_from_urdf(
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

  // Construct trajectory message
  trajectory_.joint_names.clear();
  trajectory_.points.clear();

  for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      it++)
  {
    trajectory_.joint_names.push_back(it->getJoint().getName());
  }

  trajectory_msgs::JointTrajectoryPoint single_point;
  single_point.time_from_start = ros::Duration(0.005);
  single_point.positions.resize(n_dof_);
  single_point.velocities.resize(n_dof_);
  std::fill(single_point.positions.begin(),single_point.positions.end(),0.0);
  std::fill(single_point.velocities.begin(),single_point.velocities.end(),0.0);
  trajectory_.points.push_back(single_point);


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
        250,
        1.0E-6));

  // Zero out torque data
  torques_.data.setZero();
  positions_.q.data.setZero();
  positions_.qdot.data.setZero();
  positions_des_.q.data.setZero();
  positions_des_.qdot.data.setZero();

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_des_);
  torques_out_port_.setDataSample(torques_);
  trajectories_out_port_.setDataSample(trajectory_);

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

  ROS_DEBUG_STREAM("Unwrapped angles: "<<(positions_des_.q.data.transpose)());

  // Unwrap angles
  for(unsigned int i=0; i<n_dof_; i++) {
    if(positions_des_.q(i) > 0) {
      positions_des_.q(i) = fmod(positions_des_.q(i)+M_PI,2.0*M_PI)-M_PI;
    } else {
      positions_des_.q(i) = fmod(positions_des_.q(i)-M_PI,2.0*M_PI)+M_PI;
    }
  }

  ROS_DEBUG_STREAM("Wrapped angles: "<<(positions_des_.q.data.transpose()));

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

  // Send traj target
  if(trajectories_out_port_.connected()) {
    trajectory_.header.stamp = ros::Time(0,0);//util::ros_rt_now() + ros::Duration(1.0);

    for(size_t i=0; i<n_dof_; i++) {
      trajectory_.points[0].positions[i] = positions_des_.q(i);
      trajectory_.points[0].velocities[i] = positions_des_.qdot(i);
    }

    trajectories_out_port_.write( trajectory_ );
  }

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


