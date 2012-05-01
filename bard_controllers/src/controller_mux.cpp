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
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_common/util.h>
#include <bard_controllers/controller_mux.h>

#include <boost/math/special_functions/fpclassify.hpp>

using namespace bard_controllers;

ControllerMux::ControllerMux(std::string const& name) :
  RTT::TaskContext(name)
  // RTT properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,joint_state_throttle_period_(0.1)
  // Internal members
  ,n_dof_(0)
  ,controller_torques_()
  ,positions_()
  ,torques_()
  ,enabled_(true)
  ,config_cmd_()
  ,joint_state_()
  ,joint_state_throttle_(joint_state_throttle_period_)
{

  // Declare properties
  this->addProperty("robot_description",robot_description_)
     .doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_)
    .doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure RTT ports
  this->ports()->addEventPort("config_input", config_input_)
    .doc("Input Event port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("state_output", state_output_)
    .doc("Output port: nx1 vector of joint positions. (n joints)");

  this->ports()->addEventPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_)
    .doc("Output port: sensor_msgs/JointState of commanded joint state.");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");

  // Configure operations
  this->addOperation("load", &ControllerMux::load_controller, this, RTT::OwnThread)
    .doc("Add a controller to the controller mux.")
    .arg("name","Name of controller to load.");

  this->addOperation("unload", &ControllerMux::unload_controller, this, RTT::OwnThread)
    .doc("Remove a controller from the controller mux.")
    .arg("name","Name of controller to unload.");

  this->addOperation("enable", &ControllerMux::enable, this, RTT::OwnThread)
    .doc("Enable multiplexer (output non-zero torques).");

  this->addOperation("disable", &ControllerMux::disable, this, RTT::OwnThread)
    .doc("Disable multiplexer (output zero torques).");

  this->addOperation("toggleControllers", &ControllerMux::toggle_controllers, this, RTT::ClientThread)
    .doc("Enable and disable controllers by name.")
    .arg("enable_controllers","Array of names of controllers to enable.")
    .arg("disable_controllers","Array of names of controllers to disable.");

  this->addOperation("listControllers", &ControllerMux::list_controllers, this, RTT::ClientThread)
    .doc("List the currently enabled and disabled controllers.");

  // Initialize properties from rosparam
  bard_common::util::load_rosparam_and_refresh(this);
}

bool ControllerMux::configureHook()
{
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  if(!bard_common::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }

  // Initialize joint arrays
  torques_.resize(n_dof_);
  positions_.resize(n_dof_);
  
  // Construct ros JointState message with the appropriate joint names
  bard_common::util::joint_state_from_kdl_chain(kdl_chain_, joint_state_);

  // Prepare data samples
  torques_out_port_.setDataSample(torques_);
  joint_state_out_port_.setDataSample(joint_state_);

  return true;
}

bool ControllerMux::startHook()
{
  std::cerr<<"Starting controller mux!"<<std::endl;
  return true;
}

void ControllerMux::updateHook()
{
  // Check configure input for a new configure command
  if( config_input_.read( config_cmd_ ) == RTT::NewData ) {
    // Update the properties of the controllers referenced in the command
  }
  
  // Read in the current joint positions
  positions_in_port_.readNewest( positions_ );

  // Zero out the output torques
  torques_.data.setZero();
 
  // Read in all control inputs
  for(ControllerInterface_iter it = controller_interfaces_.begin();
      it != controller_interfaces_.end();
      it++) 
  {
    // Read input from this controller
    if(it->second->in_port.readNewest(controller_torques_)) {
      // Store this control input
      it->second->last_torques.data = controller_torques_.data;
      if( it->second->enabled ) {
        // Add this control input to the output torques
        for(unsigned int i=0; i < it->second->dof && i < n_dof_; i++) {
          if(boost::math::isfinite(controller_torques_(i)) && !boost::math::isnan(controller_torques_(i))) {
            torques_(i) += controller_torques_(i);
          }
        }
      }
    }
  }

  // Only send non-zero torques if enabled
  if(enabled_) {
    // Send torque command
    torques_out_port_.write( torques_ );
  } else {
    KDL::JntArray zero_array(n_dof_);
    zero_array.data.setZero();
    torques_out_port_.write( zero_array ); 
  }
  
  // Copy the command into a sensor_msgs/JointState message
  if( joint_state_throttle_.ready(joint_state_throttle_period_)) {
    joint_state_.header.stamp = ros::Time::now();
    for(unsigned int i=0; i<n_dof_; i++) {
      joint_state_.position[i] = positions_.q(i);
      joint_state_.velocity[i] = positions_.qdot(i);
      joint_state_.effort[i] = torques_(i);
    }
    joint_state_out_port_.write( joint_state_ );
  } 
}

void ControllerMux::stopHook()
{

}

void ControllerMux::cleanupHook()
{
  // Unload all controllers
  for(ControllerInterface_iter it = controller_interfaces_.begin();
      it != controller_interfaces_.end(); it++) 
  {
    std::cerr<<"Deleting controller interface port for "<<it->first<<std::endl;
    it->second->in_port.disconnect();
    this->ports()->removePort(it->first);
    delete it->second;
  }
  controller_interfaces_.clear();
}

void ControllerMux::enable()
{
  enabled_ = true;
}

void ControllerMux::disable()
{
  enabled_ = false;
}

void ControllerMux::load_controller(std::string name)
{
  // Create a controller interface
  ControllerInterface *interface  = new ControllerInterface();
  interface->dof = n_dof_;
  interface->enabled = false;
  // Add this interface port to the task
  this->ports()->addPort(name, interface->in_port).doc("Input torques from controller \""+name+"\"");
  // Add interface to the map of controller interfaces
  controller_interfaces_[name]=interface;
}

void ControllerMux::unload_controller(std::string name)
{
  if(controller_interfaces_.find(name) != controller_interfaces_.end()) {
    delete controller_interfaces_.find(name)->second;
    controller_interfaces_.erase(name);
  }
}

void ControllerMux::toggle_controllers(
    std::vector<std::string> enable_controllers, 
    std::vector<std::string> disable_controllers) 
{
  // Enable some controllers
  for(std::vector<std::string>::iterator it = enable_controllers.begin();
      it != enable_controllers.end(); it++) 
  {
    if(controller_interfaces_.find(*it) != controller_interfaces_.end()) {
      controller_interfaces_.find(*it)->second->enabled = true;
    }
  }

  // Disable some controllers
  for(std::vector<std::string>::iterator it = disable_controllers.begin();
      it != disable_controllers.end(); it++) 
  {
    if(controller_interfaces_.find(*it) != controller_interfaces_.end()) {
      controller_interfaces_.find(*it)->second->enabled = false;
    }
  }
}

void ControllerMux::list_controllers()
{
  ROS_INFO_STREAM("Controller multiplexer is "<<((enabled_)?("ENABLED"):("DISABLED")));
  ROS_INFO("  Enabled controllers:");
  for(ControllerInterface_iter it = controller_interfaces_.begin();
      it != controller_interfaces_.end(); it++) 
  {
    if(it->second->enabled) {
      ROS_INFO_STREAM("      "<<it->first<<": "<<it->second->last_torques.data.transpose());
    }
  }
  ROS_INFO("  Disabled controllers:");
  for(ControllerInterface_iter it = controller_interfaces_.begin();
      it != controller_interfaces_.end(); it++) 
  {
    if(!it->second->enabled) {
      ROS_INFO_STREAM("      "<<it->first<<": "<<it->second->last_torques.data.transpose());
    }
  }
  ROS_INFO_STREAM("  Controller command: "<<torques_.data.transpose());
}
