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

#include <Eigen/Dense>

#include <ros/ros.h>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <sensor_msgs/JointState.h>

#ifdef __XENO__
#include <leoCAN/RTSocketCAN.h>
#else
#warning This component needs the xenomai libraries to run!
#endif

#include <barrett_hw/wam.h>

using namespace barrett_hw;

WAM::WAM(ros::NodeHandle nh) :
  barrett_model::WAMInterface(nh)
  ,can_dev_name_("")
  ,canbus_(NULL)
  ,robot_(NULL)
  ,calibrated_(false)
{
  ROS_INFO_STREAM("WAM component constructed !");
}

bool WAM::configure()
{
  using namespace terse_roscpp;

  // Initialize properties from rosparam
  try {
    this->load_params();
    require_param(nh_,"can_dev_name",can_dev_name_,
                  "The CANBus device name (rtcan0, rtcan1, etc).");
  } catch( ros::InvalidParameterException &ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  // Initialize the arm kinematics from the robot description
  if(!this->init_kinematics())
    return false;
  
  // Initialize the ros_control interfaces
  if(!this->register_hardware_interfaces())
    return false;
  
  // Initialize the ROS Control hardware interface
  this->register_hardware_interfaces();

  // Try to connect and initialize hardware
  try{
    // Construct CAN structure
#ifdef __XENO__
    canbus_.reset(new leoCAN::RTSocketCAN(can_dev_name_, leoCAN::CANBus::RATE_1000 ));
#else
    // TODO: port a non-realtime canbus to leoCAN
    ROS_FATAL("This component cannot be used without the xenomai libraries and rtsocketcan!");
    return false;
#endif

    // Open the canbus
    if( canbus_->Open() != leoCAN::CANBus::ESUCCESS ){
      ROS_ERROR_STREAM("Failed to open CAN device \""<<can_dev_name_<<"\"");
      throw std::exception();
    }

    // Construct WAM structure
    robot_.reset(new barrett_direct::WAM(canbus_.get(), (barrett_direct::WAM::Configuration)n_dof_));

    // Initialize the WAM robot
    if( robot_->Initialize() != barrett_direct::WAM::ESUCCESS ){
      ROS_ERROR_STREAM("Failed to initialize WAM");
      throw std::exception();
    }
  } catch(std::exception &ex) {
    // Free the device handles
    this->cleanup_internal();
    return false;
  }

  ROS_INFO_STREAM("WAM connected on CAN device \""<<can_dev_name_<<"\"!");

  run_state_ = WAM::CONFIGURED;

  return true;
}

bool WAM::start()
{
  // Check calibration
  if(0 && !calibrated_) {
    // Set the joints to the calibration position
    calibrated_ = this->calibrate_position(initial_positions_);
    // Return if we can't calibrate
    if(!calibrated_) {
      return false;
    }
  }

  // Set the robot to Activated
  if( robot_->SetMode(barrett_direct::WAM::MODE_ACTIVATED) != barrett_direct::WAM::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to ACTIVATE WAM Robot on CAN device \""<<can_dev_name_<<"\"");
    return false;
  }

  ROS_INFO_STREAM("WAM started on CAN device \""<<can_dev_name_<<"\"!");

  run_state_ = WAM::STARTED;

  return true;
}

void WAM::read(const ros::Time time, const ros::Duration period)
{
  // Get joint positions
  if( robot_->GetPositions( joint_state_new_.q.data ) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Failed to get positions of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }

  // Compute joint velocities
  for(unsigned int i=0; i<n_dof_; i++) {
    joint_state_.qdot(i) = 
      (joint_state_new_.q(i) - joint_state_.q(i))
      /period.toSec();
  }

  // Update positions
  joint_state_.q = joint_state_new_.q;

  // If not calibrated, also compute the motor angles
  if( !calibrated_) {
    if( robot_->GetPositionOffsets( resolver_angles_.data ) != barrett_direct::WAM::ESUCCESS) {
      ROS_ERROR_STREAM("Failed to get positions of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
    }
  }
}

void WAM::write(const ros::Time time, const ros::Duration period)
{
  // Apply torque limits
  for(unsigned int i=0; i<n_dof_; i++) {
    if(fabs(torques_(i)) > torque_limits_[i]) {
      ROS_WARN("Commanded torques exceeded safety limits! They have been truncated.");
      // Truncate this joint torque
      torques_(i) = std::max(std::min(torques_(i),
                                      torque_limits_[i]),
                             -1.0*torque_limits_[i]);
    }
  }

  // Send the torques
  if( robot_->SetTorques( torques_.data ) != barrett_direct::WAM::ESUCCESS ) {
    ROS_ERROR_STREAM("Failed to set torques of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }
}

void WAM::stop()
{
  // Set the robot to IDLE
  if( robot_->SetMode(barrett_direct::WAM::MODE_IDLE) != barrett_direct::WAM::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to IDLE WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }

  run_state_ = WAM::CONFIGURED;
}

void WAM::cleanup()
{
  // Close the CANBus
  if( canbus_->Close() != leoCAN::CANBus::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to close CAN device \""<<can_dev_name_<<"\"");
  }

  // Reset calibration flag
  calibrated_ = false;

  // Free the device handles
  this->cleanup_internal();

  run_state_ = WAM::IDLE;
}

bool WAM::calibrate_position(std::vector<double> &actual_positions)
{
  // Make sure we have a connection to the robot
  if(this->is_configured()) {
    // Assign the positions to the current robot configuration
    if(robot_->SetPositions(Eigen::Map<Eigen::VectorXd>(&actual_positions[0],n_dof_))
        != barrett_direct::WAM::ESUCCESS)
    {
      ROS_ERROR_STREAM("Failed to calibrate encoders!");
      return false;
    }

    // Set the current positions to the initial positions
    for(size_t i=0; i<actual_positions.size(); i++) {
      joint_state_.q(i) = actual_positions[i];
    }

    ROS_INFO("Calibrated encoders.");
    return true;
  } else {
    ROS_ERROR_STREAM("Cannot calibrate encoders! The WAM control task on device "<<can_dev_name_<<" is not configured1");
  }

  return false;
}

void WAM::cleanup_internal()
{
  // Reset the scoped pointers
  robot_.reset(NULL);
  canbus_.reset(NULL);
}

void WAM::set_velocity_warn(unsigned int thresh)
{
  if(!this->is_configured() || robot_->SetVelocityWarning(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set velocity warning threshold.");
  }
}
void WAM::set_velocity_fault(unsigned int thresh)
{
  if(!this->is_configured() || robot_->SetVelocityFault(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set velocity fault threshold.");
  }
}
void WAM::set_torque_warn(unsigned int thresh)
{
  if(!this->is_configured() || robot_->SetTorqueWarning(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set torque warning threshold.");
  }
}
void WAM::set_torque_fault(unsigned int thresh)
{
  if(!this->is_configured() || robot_->SetTorqueFault(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set torque fault threshold.");
  }
}
