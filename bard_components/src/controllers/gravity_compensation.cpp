
#include <iostream>

#include <Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>

#include <leoCAN/RTSocketCAN.h>
#include <barrett_direct/WAM.h>

#include <bard_components/util.h>
#include <bard_components/controllers/gravity_compensation.h>

using namespace bard_components;

GravityCompensation::GravityCompensation(string const& name) :
  TaskContext(name)
  // Operation Callers
  ,get_robot_properties_("get_robot_properties")
  // Properties
  ,root_joint_("")
  ,tip_joint_("")
  // Robot properties
  ,n_wam_dof_(7)
  ,robot_description_("")
  ,joint_prefix_("")
  // Working variables
  ,kdl_tree_()
  ,kdl_chain_()
  ,id_solver_(NULL)
  ,positions_(n_wam_dof_)
  ,velocities_(n_wam_dof_)
  ,accelerations_(n_wam_dof_)
  ,torques_(n_wam_dof_)
  ,wrenches_(n_wam_dof_)
{
  // Declare properties
  this->addProperty("root_joint",root_joint_).doc("The root joint for the controller.");
  this->addProperty("tip_joint",tip_joint_).doc("The tip joint for the controller.");

  // Configure data ports
  this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_).doc("Output port: sensor_msgs/JointState of commanded joint state.");

  // Configure operations
  this->requires("robot_properties")
    ->addOperationCaller(get_robot_properties_);
}

bool GravityCompensation::configureHook()
{
  // Make sure this controller has been connected to a WAM robot
  if(!this->requires("robot_properties")->ready()) {
    std::cerr<<"WARNING: no connection to robot component, needed for robot properties"<<std::endl;
    return false;
  }

  // Get properties from wam robot
  get_robot_properties_(n_wam_dof_, robot_description_, joint_prefix_);

  // Construct an URDF model from the xml string
  urdf::Model urdf_model;
  urdf_model.initString(robot_description_);

  // Get root link
  std::string root_name = urdf_model.getRoot()->name;

  // Get a KDL tree from the robot URDF
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  // Populate the KDL chain
  kdl_tree_.getChain(
      joint_prefix_+"/YawJoint",
      joint_prefix_+"/LowerWristYawJoint",
      kdl_chain_);

  // Create chainsolver
  id_solver_.reset(
      new KDL::ChainIdSolver_RNE(
        kdl_chain_,
        KDL::Vector(0,0,-9.8)));

  // Resize working vectors
  positions_.resize(n_wam_dof_);
  velocities_.resize(n_wam_dof_);
  accelerations_.resize(n_wam_dof_);
  torques_.resize(n_wam_dof_);
  wrenches_.resize(kdl_chain_.getNrOfSegments());

  // Zero out torque data
  torques_.data.setZero();

  // Construct ros JointState message
  util::init_wam_joint_state(
      n_wam_dof_,
      prefix_,
      joint_state_);

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);
  joint_state_out_port_.setDataSample(joint_state_);

  return true;
}

bool GravityCompensation::startHook()
{
  return true;
}

void GravityCompensation::updateHook()
{
  // Compute inverse dynamics
  // This computes the torques on each joint of the arm as a function of
  // the arm's joint-space position, velocities, accelerations, external
  // forces/torques and gravity.
  id_solver_->CartToJnt(
      positions_,
      velocities_,
      accelerations_,
      ext_wrenches_,
      torques_);
  // Send joint positions
  torques_out_port_.write( torques_ );
}

void GravityCompensation::stopHook()
{
}

void GravityCompensation::cleanupHook()
{
}
