#ifndef __CHAIN_IK_SOLVER_BFGS_H
#define __CHAIN_IK_SOLVER_BFGS_H

namespace bard_kinematics {
  class ChainIKSolverBFGS {
  public:
    ChainIKSolverBFGS(
        urdf::Model urdf_model,
        std::string root_link,
        std::string tip_link,
        RTT::os::TimeService::Seconds time_limit) :
      udrf_model_(urdf_model)
      ,root_link_(root_link)
      ,tip_link_(tip_link)
      ,time_limit_(time_limit)
    {
      // Get a KDL tree from the robot URDF
      if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
      }

      // Populate the KDL chain
      if(!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain_))
      {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_link<<" --> "<<tip_link);
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
      }

    bool cart_to_joint(
        Eigen::VectorXd initial_positions,
        Eigen::Matrix) {
      
    }
  };
}

#endif // ifndef __CHAIN_IK_SOLVER_BFGS_H
