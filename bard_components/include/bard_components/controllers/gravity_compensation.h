#ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H
#define __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>

namespace bard_components {
  namespace controllers {
    class GravityCompensation : public RTT::TaskContext
    {
      // RTT Properties
      std::string root_joint_;
      std::String tip_joint_;

      // RTT Ports
      RTT::OutputPort<KDL::JntArray> torques_out_port_;
      RTT::OutputPort<sensor_msgs::JointState> joint_state_out_port_;

      // RTT Operations
      OperationCaller<void(int&,std::string&,std::string&)> get_robot_properties_;

    public:
      GravityCompensation(string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

    private:
      //  Robot configuration
      int n_wam_dof_;
      std::string robot_model_xml_;
      std::string joint_prefix_;

      // Working variables
      KDL::Tree kdl_tree_;
      KDL::Chain kdl_chain_;
      boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

      KDL::JntArray positions_;
      KDL::JntArray velocities_;
      KDL::JntArray accelerations_;
      KDL::JntArray torques_;

      KDL::Wrenches ext_wrenches_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

