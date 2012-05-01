#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_common/util.h>
#include <bard_controllers/controllers/joint_pid.h>

using namespace bard_controllers::controllers;

Cartesian::Cartesian(string const& name) :
  TaskContext(name)
  // Properties
  ,n_arm_dof_(7)
  ,robot_description_("")
  ,joint_prefix_("")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame()
  // Working variables
  ,kdl_tree_()
  ,kdl_chain_()
  ,des_positions_()
  ,des_velocities_()
  ,positions_()
  ,velocities_()
  ,torques_(n_arm_dof_)
{
  // Declare properties
  this->addProperty("n_arm_dof",n_arm_dof_).doc("The number of degrees-of-freedom of the WAM robot (4 or 7).");
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("joint_prefix",joint_prefix_).doc("The joint name prefix used in the WAM URDF.");

  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("target_frame",joint_prefix_).doc("The target frame to track wotj tip_link.");

  // Configure data ports
  this->ports()->addPort("des_positions_in", des_positions_in_port_).doc("Input port: nx1 vector of desired joint positions. (n joints)");
  this->ports()->addPort("des_velocities_in", des_velocities_in_port_).doc("Input port: nx1 vector of desired joint velocities. (n joints)");
  this->ports()->addPort("positions_in", positions_in_port_).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("velocities_in", velocities_in_port_).doc("Input port: nx1 vector of joint velocities. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");
  
  // Initialize properties from rosparam
  bard_common::util::load_rosparam_and_refresh(this);
}

bool Cartesian::configureHook()
{
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
  if(!kdl_tree_.getChain(
        joint_prefix_+"/"+root_link_,
        joint_prefix_+"/"+tip_link_,
        kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: "
        <<joint_prefix_<<"/"<<root_link_
        <<" --> "
        <<joint_prefix_<<"/"<<tip_link_
        <<std::endl
        <<"  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints"
        <<"  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments"
        <<"  The segments are:"
        );
    return false;
  }

  // Initialize cartesian position solver
  kdl_fk_solver_pos_.reset(new ChainFkSolverPos_recursive(kdl_chain_));

  // Resize working vectors
  des_positions_.resize(n_arm_dof_);
  des_velocities_.resize(n_arm_dof_);
  positions_.resize(n_arm_dof_);
  velocities_.resize(n_arm_dof_);
  torques_.resize(n_arm_dof_);

  // Zero out torque data
  torques_.data.setZero();

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);

  return true;
}

bool Cartesian::startHook()
{
  return true;
}

void Cartesian::updateHook()
{
  // Read in the goal joint positions & velocities
  if(des_positions_in_port_.readNewest( des_positions_new_ ) == RTT::NewData) {
    des_positions_ = des_positions_new_;
  }
  if(des_velocities_in_port_.readNewest( des_velocities_new_ ) == RTT::NewData) {
    des_velocities_ = des_velocities_new_;
  }

  // Read in the current joint positions
  positions_in_port_.read( positions_ );
  velocities_in_port_.read( velocities_ );

  for(int i=0; i<n_arm_dof_; i++) {
    torques_(i) =
      kp_[i]*(des_positions_(i) - positions_(i))
      + kd_[i]*(des_velocities_(i) - velocities_(i));
  }
 
  // Send joint positions
  torques_out_port_.write( torques_ );
}

void Cartesian::stopHook()
{
}

void Cartesian::cleanupHook()
{
}

