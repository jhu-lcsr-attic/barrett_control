#ifndef __BARD_COMPONENTS_CONTROLLERS_CARTESIAN
#define __BARD_COMPONENTS_CONTROLLERS_CARTESIAN

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace bard_components {
  namespace controllers {
    class Cartesian : public RTT::TaskContext
    {
      // RTT Properties
      int n_arm_dof_;
      std::string robot_description_;
      std::string joint_prefix_;
      std::string root_link_;
      std::string tip_link_;
      std::string target_frame_

      // RTT Ports
      RTT::InputPort<KDL::JntArray> des_positions_in_port_;
      RTT::InputPort<KDL::JntArray> des_velocities_in_port_;

      RTT::InputPort<KDL::JntArray> positions_in_port_;
      RTT::InputPort<KDL::JntArray> velocities_in_port_;
      RTT::OutputPort<KDL::JntArray> torques_out_port_;

    public:
      Cartesian(std::string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

    private:

      // Working variables
      KDL::Tree kdl_tree_;
      KDL::Chain kdl_chain_;

      KDL::JntArray des_positions_new_;
      KDL::JntArray des_velocities_new_;


      boost::scoped_ptr<KDL::ChainFkSolverPos> kdl_fk_solver_pos_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_CARTESIAN


