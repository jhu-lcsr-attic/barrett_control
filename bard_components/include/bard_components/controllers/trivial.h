#ifndef __BARD_COMPONENTS_CONTROLLERS_TRIVIAL_H
#define __BARD_COMPONENTS_CONTROLLERS_TRIVIAL_H

#include <Eigen/Dense>
#include <kdl/jntarray.hpp>

#include <barrett_direct/WAM.h>
#include <leoCAN/RTSocketCAN.h>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <iostream>

namespace bard_components {
  namespace controllers {
    class Trivial : public RTT::TaskContext
    {
      // RTT Interface
      RTT::OutputPort<KDL::JntArray> torques_out_port_;

      // See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
      // See: http://www.orocos.org/forum/orocos/orocos-users/some-info-eigen-and-orocos
      // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    public:
      Trivial(string const& name) :
        TaskContext(name),
        n_wam_dof_(7),
        torques_(n_wam_dof_)
      {
        // Configure data ports
        this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");

        torques_.data.setZero();

        // Prepare ports for realtime processing
        KDL::JntArray joints_sample(7);
        torques_out_port_.setDataSample(joints_sample);
      }

      bool configureHook() {
        return true;
      }

      bool startHook() {
        return true;
      }

      void updateHook() {
        // Send joint positions
        torques_out_port_.write( torques_ );
      }

      void stopHook() {
      }

      void cleanupHook() {
      }

    private:
      // Configuration properties
      int n_wam_dof_;
      // Working variables
      KDL::JntArray torques_;

    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_TRIVIAL_H
