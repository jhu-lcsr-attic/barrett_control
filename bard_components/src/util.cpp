
#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>

#include <rtt/RTT.hpp>

#include <bard_components/util.h>

using namespace bard_components;

bool util::initialize_kinematics_from_urdf(
    const std::string &robot_description,
    const std::string &root_link,
    const std::string &tip_link,
    unsigned int &n_dof,
    KDL::Chain &kdl_chain,
    KDL::Tree &kdl_tree,
    urdf::Model &urdf_model)
{
  // Construct an URDF model from the xml string
  urdf_model.initString(robot_description);

  // Get a KDL tree from the robot URDF
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  // Populate the KDL chain
  if(!kdl_tree.getChain(root_link, tip_link, kdl_chain))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
    ROS_ERROR_STREAM("  "<<root_link<<" --> "<<tip_link);
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree.getSegments();
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
  n_dof = kdl_chain.getNrOfJoints();

  return true;
}


void util::joint_state_from_kdl_chain(
    const KDL::Chain &kdl_chain,
    sensor_msgs::JointState &joint_state)
{
  // Construct blank joint state message
  joint_state = sensor_msgs::JointState();

  // Add joint names
  for(std::vector<KDL::Segment>::const_iterator it=kdl_chain.segments.begin();
      it != kdl_chain.segments.end();
      it++)
  {
    joint_state.name.push_back(it->getJoint().getName());
  }

  // Get the #DOF
  unsigned int n_dof = kdl_chain.getNrOfJoints();

  // Resize joint vectors
  joint_state.position.resize(n_dof);
  joint_state.velocity.resize(n_dof);
  joint_state.effort.resize(n_dof);
}

ros::Time util::ros_rtt_now(); {
  return ros::Time(0,RTT::os::TimeService::Instance()->getNSecs())
}
