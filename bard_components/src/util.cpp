
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include <bard_components/util.h>

using namespace bard_components;

bool util::initialize_kinematics_from_urdf(
    const std::string &robot_description,
    const std::string &root_link,
    const std::string &tip_link,
    KDL::Tree &kdl_tree,
    KDL::Chain &kdl_chain,
    unsigned int &n_dof)
{
  // Construct an URDF model from the xml string
  urdf::Model urdf_model;
  urdf_model.initString(robot_description_);

  // Get a KDL tree from the robot URDF
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  // Populate the KDL chain
  if(!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
    ROS_ERROR_STREAM("  "<<root_link_<<" --> "<<tip_link_);
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("  The segments are:");

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

  // Store the number of degrees of freedom of the chain
  n_dof_ = kdl_chain_.getNrOfJoints();

  return true;
}


void util::joint_state_from_kdl_chain(
    const KDL::Chain &chain,
    sensor_msgs::JointState &joint_state)
{
  // Construct blank joint state message
  joint_state_ = sensor_msgs::JointState();

  // Add joint names
  for(std::vector<KDL::Segment>::iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      it++)
  {
    joint_state_.name.push_back(it->name);
  }

  // Resize joint vectors
  joint_state.position.resize(n_dof_);
  joint_state.velocity.resize(n_dof_);
  joint_state.effort.resize(n_dof_);
}
