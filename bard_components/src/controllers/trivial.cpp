
#include <bard_components/controllers/trivial.h>

using namespace bard_components::controllers;

Trivial::Trivial(string const& name) :
  TaskContext(name),
  n_arm_dof_(0),
  positions_(),
  torques_()
{
  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
}

bool Trivial::configureHook() {
  return true;
}

bool Trivial::startHook() {
  // Read in a sample to resize the torques appropriately
  if(!positions_in_port_.connected()) {
    positions_in_port_.read(positions_);
    n_arm_dof_ = positions_.q.rows();
  } else {
    ROS_ERROR("Port \"positions_in\" not connected. It is needed to appropriately allocate the controller torque command.");
    return false;
  }

  // Resize and zero out the torques
  torques_.resize(n_arm_dof_);
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
