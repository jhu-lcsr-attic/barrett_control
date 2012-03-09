
#include <bard_components/controllers/trivial.h>

using namespace bard_components::controllers;

Trivial::Trivial(string const& name) :
  TaskContext(name),
  n_wam_dof_(7),
  torques_(n_wam_dof_)
{
  // Configure data ports
  this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");
}

bool Trivial::configureHook() {
  // Resize and zero out the torques
  torques_.resize(n_wam_dof_);
  torques_.data.setZero();

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);
  return true;
}

bool Trivial::startHook() {
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
