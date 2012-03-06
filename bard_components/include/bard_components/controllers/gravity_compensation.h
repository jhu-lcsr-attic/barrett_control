#ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H
#define __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

#include <Eigen/Dense>
#include <kdl/jntarray.hpp>

#include <barrett_direct/WAM.h>
#include <leoCAN/RTSocketCAN.h>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl_parser/kdl_parser.hpp>

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
        robot_model_xml_(""),
        torques_(n_wam_dof_)
      {
        // Configure data ports
        this->ports()->addPort("torques_out", torques_out_port_).doc("Output port: nx1 vector of joint torques. (n joints)");

        // Zero out torque data
        torques_.data.setZero();

        // Prepare ports for realtime processing
        torques_out_port_.setDataSample(torques_);
      }

      bool configureHook() {
        // Working variables
        KDL::Tree tree;
        KDL::Chain chain;

        // Construct an URDF model from the xml string
        urdf::Model urdf_model;
        urdf_model.initString(robot_model_xml_);

        // Get root link
        std::string root_name = urdf_model.getRoot().name;

        // Get a KDL tree from the robot URDF
        if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)){
          ROS_ERROR("Failed to construct kdl tree");
          return false;
        }

        // Populate the KDL chain
        tree.getChain(joint_prefix_+"/YawJoint",joint_prefix_+"/LowerWristYawJoint",chain);

        // Create chainsolver
        KDL::ChainIdSolver_RNE ChainIdSolver_RNE(

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
      std::string robot_model_xml_;

      // Working variables
      KDL::JntArray torques_;
    };
  }
}


#endif // ifndef __BARD_COMPONENTS_CONTROLLERS_GRAVITY_COMPENSATION_H

