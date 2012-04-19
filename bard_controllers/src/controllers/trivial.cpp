
#include <ros/ros.h>

#include <bard_common/util.h>
#include <bard_controllers/controllers/trivial.h>

using namespace bard_controllers::controllers;

Trivial::Trivial(string const& name) :
  TaskContext(name),
  root_link_(""),
  tip_link_(""),
  n_dof_(0),
  positions_(),
  torques_()
{
  // Configure properties
  this->addProperty("robot_description",robot_description_)
     .doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
  
  // Initialize properties from rosparam
  bard_common::util::load_rosparam_and_refresh(this);
}

bool Trivial::configureHook() {
  return true;
}

bool Trivial::startHook() {
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  urdf::Model urdf_model;
  if(!bard_common::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain, kdl_tree, urdf_model))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }

  // Resize and zero out the torques
  torques_.resize(n_dof_);
  torques_.data.setZero();

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);

  return true;
}

void Trivial::updateHook() {
  // Send joint torques (always zero)
  torques_out_port_.write( torques_ );
}

void Trivial::stopHook() {
}

void Trivial::cleanupHook() {
}
