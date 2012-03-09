
#include <sensor_msgs/JointState.h>

#include <bard_components/util.h>

using namespace bard_components;

void util::init_wam_joint_state(
    const int n_wam_dof,
    const std::string &joint_prefix,
    sensor_msgs::JointState &joint_state)
{
  // Add joint names
  joint_state.name.push_back(joint_prefix+"/YawJoint");
  joint_state.name.push_back(joint_prefix+"/ShoulderPitchJoint");
  joint_state.name.push_back(joint_prefix+"/ShoulderYawJoint");
  joint_state.name.push_back(joint_prefix+"/ElbowJoint");
  if(n_wam_dof == 7) {
    joint_state.name.push_back(joint_prefix+"/UpperWristYawJoint");
    joint_state.name.push_back(joint_prefix+"/UpperWristPitchJoint");
    joint_state.name.push_back(joint_prefix+"/LowerWristYawJoint");
  }

  // Resize joint vectors
  joint_state.position.resize(n_wam_dof);
  joint_state.velocity.resize(n_wam_dof);
  joint_state.effort.resize(n_wam_dof);
}
