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
      std::string root_joint_;
      std::string tip_joint_;
      RTT::os::TimeService::Seconds joint_state_throttle_period_;

      // RTT Ports
      RTT::OutputPort<KDL::JntArray> torques_out_port_;
      RTT::OutputPort<sensor_msgs::JointState> joint_state_out_port_;

      // RTT Operations
      RTT::OperationCaller<void(int&,std::string&,std::string&)> get_robot_properties_;

    public:
      GravityCompensation(std::string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

    private:
      //  Robot configuration
      int n_wam_dof_;
      std::string robot_description_;
      std::string joint_prefix_;

      // Working variables
      KDL::Tree kdl_tree_;
      KDL::Chain kdl_chain_;
      boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

      KDL::Wrenches ext_wrenches_;

      KDL::JntArray positions_;
      KDL::JntArray velocities_;
      KDL::JntArray accelerations_;
      KDL::JntArray torques_;

      sensor_msgs::JointState joint_state_;
      RTT::os::TimeService::ticks joint_state_pub_time_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

