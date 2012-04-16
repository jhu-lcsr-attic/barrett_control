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
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <sensor_msgs/JointState.h>

#ifdef __XENO__
#include <leoCAN/RTSocketCAN.h>
#else
#warning This component needs the xenomai libraries to run!
#endif

#include <bard_common/util.h>
#include <bard_hardware/wam.h>

using namespace bard_common;
using namespace bard_hardware;

WAM::WAM(string const& name) :
  WAMInterface(name)
  // RTT Properties
  ,can_dev_name_("")
  // Internal variables
  ,canbus_(NULL)
  ,robot_(NULL)
  ,needs_calibration_(true)
{
  // Initialize all the RTT properties/ports/services
  init_rtt_interface();

  ROS_INFO_STREAM("WAM component \""<<name<<"\" constructed !");
}

bool WAM::configureHook()
{
  // Initialize the arm kinematics from the robot description
  if(!this->init_kinematics()) {
    return false;
  }

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

  return true;
}

bool WAM::startHook()
{
  // Check the data ports
  if ( !torques_in_port_.connected() ) {
    ROS_WARN_STREAM("WARNING: No connection to \"torques_in\" for WAM on \""<<can_dev_name_<<"\"!");
  }
  if ( !positions_out_port_.connected() ) {
    ROS_WARN_STREAM("WARNING: No connection to \"positions_out\" for WAM on \""<<can_dev_name_<<"\"!");
  }

  if(needs_calibration_) {
    // Set the joints to the calibration position
    this->calibrate_position(initial_positions_);
    needs_calibration_ = false;
  }

  // Set the robot to Activated
  if( robot_->SetMode(barrett_direct::WAM::MODE_ACTIVATED) != barrett_direct::WAM::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to ACTIVATE WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }

  ROS_INFO_STREAM("WAM started on CAN device \""<<can_dev_name_<<"\"!");
  return true;
}

void WAM::updateHook()
{
  // Only send joint torques if new data is coming in
  if( torques_in_port_.read( torques_ ) == RTT::NewData ) {
    // Apply torque limits
    for(unsigned int i=0; i<n_dof_; i++) {
      if(fabs(torques_(i)) > torque_limits_[i]) {
        // Truncate this joint torque
        torques_(i) = (torques_(i)>0.0)?(torque_limits_[i]):(-torque_limits_[i]);
        ROS_WARN("Commanded torques exceeded safety limits! They have been truncated.");
      }
    }
    // Send the torques
    if( robot_->SetTorques( torques_.data ) != barrett_direct::WAM::ESUCCESS ) {
      ROS_ERROR_STREAM("Failed to set torques of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
    }
  }
  
  // Get joint positions
  if( robot_->GetPositions( positions_new_.q.data ) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Failed to get positions of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }

  // Get the actual loop period
  loop_period_ = RTT::os::TimeService::Instance()->secondsSince(last_loop_time_);
  // Compute joint velocities
  for(unsigned int i=0; i<n_dof_; i++) {
    positions_.qdot(i) = (positions_new_.q(i) - positions_.q(i))/loop_period_;
  }
  // Store this time
  last_loop_time_ = RTT::os::TimeService::Instance()->getTicks();

  // Update positions
  positions_.q = positions_new_.q;

  // Send joint positions
  positions_out_port_.write( positions_ );

  // Publish joint state
  this->publish_throttled_joint_state();
}

void WAM::stopHook()
{
  // Set the robot to IDLE
  if( robot_->SetMode(barrett_direct::WAM::MODE_IDLE) != barrett_direct::WAM::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to IDLE WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }
}

void WAM::cleanupHook()
{
  // Close the CANBus
  if( canbus_->Close() != leoCAN::CANBus::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to close CAN device \""<<can_dev_name_<<"\"");
  }

  // Reset calibration flag
  needs_calibration_ = true;

  // Free the device handles
  this->cleanup_internal();
}

void WAM::calibrate_position(std::vector<double> &actual_positions)
{
  // Make sure we have a connection to the robot
  if(this->isConfigured()) {
    // Assign the positions to the current robot configuration
    if(robot_->SetPositions(Eigen::Map<Eigen::VectorXd>(&actual_positions[0],n_dof_))
        != barrett_direct::WAM::ESUCCESS)
    {
      ROS_ERROR_STREAM("Failed to calibrate encoders!");
      return;
    }

    // Set the current positions to the initial positions
    for(size_t i=0; i<actual_positions.size(); i++) {
      positions_.q(i) = actual_positions[i];
    }

    ROS_INFO("Calibrated encoders.");
  } else {
    ROS_ERROR_STREAM("Cannot calibrate encoders! The WAM control task on device "<<can_dev_name_<<" is not configured.");
  }
}

void WAM::cleanup_internal()
{
  // Reset the scoped pointers
  robot_.reset(NULL);
  canbus_.reset(NULL);
}

void WAM::set_velocity_warn(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetVelocityWarning(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set velocity warning threshold.");
  }
}
void WAM::set_velocity_fault(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetVelocityFault(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set velocity fault threshold.");
  }
}
void WAM::set_torque_warn(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetTorqueWarning(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set torque warning threshold.");
  }
}
void WAM::set_torque_fault(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetTorqueFault(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set torque fault threshold.");
  }
}
