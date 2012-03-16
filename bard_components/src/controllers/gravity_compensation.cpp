
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_components/util.h>
#include <bard_components/controllers/gravity_compensation.h>

using namespace bard_components::controllers;

GravityCompensation::GravityCompensation(string const& name) :
  TaskContext(name)
  // Properties
  ,n_arm_dof_(7)
  ,robot_description_("")
  ,joint_prefix_("")
  ,gravity_(3,0.0)
  ,root_link_("")
  ,tip_link_("")
  ,joint_state_throttle_period_(0.01)
  // Working variables
  ,kdl_tree_()
  ,kdl_chain_()
  ,id_solver_(NULL)
  ,ext_wrenches_(n_arm_dof_)
  ,positions_(n_arm_dof_)
  ,velocities_(n_arm_dof_)
  ,accelerations_(n_arm_dof_)
  ,torques_(n_arm_dof_)
{
  // Declare properties
  this->addProperty("n_arm_dof",n_arm_dof_).doc("The number of degrees-of-freedom of the WAM robot (4 or 7).");
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("joint_prefix",joint_prefix_).doc("The joint name prefix used in the WAM URDF.");

  this->addProperty("gravity",gravity_).doc("The gravity vector in the root frame.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("velocities_in", velocities_in_port_).doc("Input port: nx1 vector of joint velocities. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");
}

bool GravityCompensation::configureHook()
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

    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin();
        it != segment_map.end();
        it++ )
    {
      ROS_ERROR_STREAM( "    "<<(*it).first);
    }
  
    return false;
  }

  // Create chainsolver
  id_solver_.reset(
      new KDL::ChainIdSolver_RNE(
        kdl_chain_,
        KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));

  // Resize working vectors
  positions_.resize(n_arm_dof_);
  velocities_.resize(n_arm_dof_);
  accelerations_.resize(n_arm_dof_);
  torques_.resize(n_arm_dof_);
  ext_wrenches_.resize(kdl_chain_.getNrOfSegments());

  // Zero out torque data
  torques_.data.setZero();

  // Prepare ports for realtime processing
  torques_out_port_.setDataSample(torques_);

  return true;
}

bool GravityCompensation::startHook()
{
  return true;
}

void GravityCompensation::updateHook()
{
  // Read in the current joint positions
  positions_in_port_.read( positions_ );
  velocities_in_port_.read( velocities_ );

  // Compute inverse dynamics
  // This computes the torques on each joint of the arm as a function of
  // the arm's joint-space position, velocities, accelerations, external
  // forces/torques and gravity.
  if(id_solver_->CartToJnt(
        positions_,
        velocities_,
        accelerations_,
        ext_wrenches_,
        torques_) != 0)
  {
    std::cerr<<"ERROR: Could not compute joint torques!"<<std::endl;
  }
 
  // Send joint positions
  torques_out_port_.write( torques_ );
}

void GravityCompensation::stopHook()
{
}

void GravityCompensation::cleanupHook()
{
}
