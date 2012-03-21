#ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H
#define __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

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
    class GravityCompensation : public RTT::TaskContext
    {
      // RTT Properties
      std::string robot_description_;
      std::vector<double> gravity_;
      std::string root_link_;
      std::string tip_link_;

      // RTT Ports
      RTT::InputPort<KDL::JntArray> positions_in_port_;
      RTT::InputPort<KDL::JntArray> velocities_in_port_;
      RTT::OutputPort<KDL::JntArray> torques_out_port_;

    public:
      GravityCompensation(std::string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

    private:

      // Working variables
      int n_dof_;
      KDL::Tree kdl_tree_;
      KDL::Chain kdl_chain_;
      boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

      KDL::Wrenches ext_wrenches_;

      KDL::JntArray positions_;
      KDL::JntArray velocities_;
      KDL::JntArray accelerations_;
      KDL::JntArray torques_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

