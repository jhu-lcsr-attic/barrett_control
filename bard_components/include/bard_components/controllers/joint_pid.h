#ifndef __BARD_COMPONENTS_CONTROLLERS_JOINT_PID
#define __BARD_COMPONENTS_CONTROLLERS_JOINT_PID

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <sensor_msgs/JointState.h>

namespace bard_components {
  namespace controllers {
    class JointPID : public RTT::TaskContext
    {
      // RTT Properties
      std::string robot_description_;
      std::string root_link_;
      std::string tip_link_;
      std::vector<double> kp_, ki_, i_clamp_, kd_;
      RTT::os::TimeService::Seconds joint_state_throttle_period_;

      // RTT Ports
      RTT::InputPort<KDL::JntArrayVel> positions_in_port_;
      RTT::InputPort<KDL::JntArrayVel> positions_des_in_port_;
      RTT::OutputPort<KDL::JntArray> torques_out_port_;
      RTT::OutputPort<sensor_msgs::JointState> joint_state_out_port_;

    public:
      JointPID(std::string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      bool within_error(KDL::JntArray &pos_des);

    private:

      // Working variables
      unsigned int n_dof_;
      KDL::Chain kdl_chain_;
      KDL::Tree kdl_tree_;
      urdf::Model urdf_model_;

      KDL::JntArrayVel positions_;
      KDL::JntArrayVel positions_des_;
      KDL::JntArray p_error_;
      KDL::JntArray i_error_;
      KDL::JntArray d_error_;
      KDL::JntArray torques_;

      sensor_msgs::JointState joint_state_;
      bard_components::util::PeriodicThrottle joint_state_throttle_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_JOINT_PID

