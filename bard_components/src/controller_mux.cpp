
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_components/util.h>
#include <bard_components/controller_mux.h>

using namespace bard_components;

ControllerMux::ControllerMux(std::String const& name) :
  RTT::TaskContext(name)
  ,n_arm_dof(0)
  ,joint_prefix("")
  ,joint_state_throttle_period_(0.1)
  ,controller_torques_()
  ,torques_()
  ,enabled_(true)
  ,joint_state_()
  ,joint_state_pub_time_(0)
{

  // Declare properties
  this->addProperty("n_arm_dof",n_arm_dof_).doc("The number of degrees-of-freedom of the WAM robot (4 or 7).");
  this->addProperty("joint_prefix",joint_prefix_).doc("The joint name prefix used in the WAM URDF.");
  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_).doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure RTT ports
  this->ports()->addEventPort("config_input_", config_input_).doc("Input Event port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("state_output_", state_output_).doc("Output port: nx1 vector of joint positions. (n joints)");

  this->ports()->addPort("positions_in", positions_in_port_).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_).doc("Output port: sensor_msgs/JointState of commanded joint state.");

  // Configure operations
  this->addOperation("load", &ControllerMux::load_controller, this, RTT::OwnThread)
    .doc("Add a controller to the controller mux.")
    .arg("name","Name of controller to load.")
    .arg("dof","Number of degrees-of-freedom that the control outputs");

  this->addOperation("unload", &ControllerMux::unload_controller, this, RTT::OwnThread)
    .doc("Remove a controller from the controller mux.")
    .arg("name","Name of controller to unload.");

  this->addOperation("enable", &ControllerMux::enable, this, RTT::OwnThread)
    .doc("Enable multiplexer (output non-zero torques).");

  this->addOperation("disable", &ControllerMux::disable, this, RTT::OwnThread)
    .doc("Disable multiplexer (output zero torques).");

  this->addOperation("toggleControllers", &ControllerMux::toggle_controllers, this, RTT::OwnThread)
    .doc("Enable and disable controllers by name.")
    .arg("enable_controllers","Array of names of controllers to enable.")
    .arg("disable_controllers","Array of names of controllers to disable.");

}

bool ControllerMux::configureHook()
{
  // Initialize output structure
  torques_ = KDL::JntArray(n_arm_dof_);
  
  // Construct ros JointState message
  util::init_wam_joint_state(
      n_arm_dof_,
      joint_prefix_,
      joint_state_);

  // Prepare data sample
  joint_state_out_port_.setDataSample(joint_state_);

}

bool ControllerMux::startHook()
{

}

void ControllerMux::updateHook()
{
  // Check configure input for a new configure command
  if( config_input_.read( config_cmd_ ) == RTT::NewData ) {
    // Update the properties of the controllers referenced in the command
  }

  // Zero out the output torques
  torques_.data.setZero();
 
  // Only compute non-zero torques if enabled
  if(enabled_) {
    // Read in all control inputs
    for(ControllerInterface_iter it = controller_interfaces_.begin();
        it != enable_controllers.end(); ++it) 
    {
      // Combine control inputs based on gains
      if( it->second->enabled ) {
        // Read input from this controller
        it->second->in_port.read(controller_torques_);
        // Add this control input to the output torques
        for(int i=0; i < it->second->dof && i < dof_; i++) {
          torques_(i) += controller_torques_(i);
        }
      }
    }
  }

  // Send joint positions
  torques_out_port_.write( torques_ );
  
  // Copy the command into a sensor_msgs/JointState message
  if( RTT::os::TimeService::Instance()->secondsSince(joint_state_pub_time_) > joint_state_throttle_period_ ) {
    joint_state_.header.stamp = ros::Time::now();
    for(int i=0; i<n_arm_dof_; i++) {
      joint_state_.position[i] = positions_(i);
      joint_state_.effort[i] = torques_(i);
    }
    joint_state_out_port_.write( joint_state_ );
    joint_state_pub_time_ = RTT::os::TimeService::Instance()->getTicks();
  } 
}

void ControllerMux::stopHook()
{

}

void ControllerMux::cleanupHook()
{
  // Unload all controllers
  for(ControllerInterface_iter it = controller_interfaces_.begin();
      it != enable_controllers.end(); ++it) 
  {
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

void ControllerMux::load_controller(std::string name, int dof)
{
  // Create a controller interface
  ControllerInterface *interface  = new ControllerInterface();
  interface.dof = dof;
  interface.enabled = false;
  // Add this interface port to the task
  this->ports()->addPort(name, interface.in_port).doc("Input torques from controller \""+name+"\"");
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
    std::vector<std::string> diable_controllers) 
{
  // Enable some controllers
  for(std::vector<std::string>::iterator it = enable_controllers.begin();
      it != enable_controllers.end(); ++it) 
  {
    if(controller_interfaces_.find(*it) != controller_interfaces_.end()) {
      controller_interfaces_.find(*it)->second->enabled = true;
    }
  }

  // Disable some controllers
  for(std::vector<std::string>::iterator it = disable_controllers.begin();
      it != disable_controllers.end(); ++it) 
  {
    if(controller_interfaces_.find(*it) != controller_interfaces_.end()) {
      controller_interfaces_.find(*it)->second->enabled = false;
    }
  }
}
