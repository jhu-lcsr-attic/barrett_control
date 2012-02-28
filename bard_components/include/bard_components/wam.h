#ifndef __BARD_COMPONENTS_WAM_H
#define __BARD_COMPONENTS_WAM_H

#include <barrett_direct/WAM.h>
#include <leoCAN/RTSocketCAN.h>

#include <rtt/RTT.hpp>
#include <iostream>

namespace bard_components {
  class WAM
    : public RTT::TaskContext
  {
    // RTT Interface
    RTT::InputPort<Eigen::VectorXd> torques_in_port_;
    RTT::OutputPort<Eigen::VectorXd> positions_out_port_;

    // See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    // See: http://www.orocos.org/forum/orocos/orocos-users/some-info-eigen-and-orocos
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    WAM(string const& name, can_dev_name WAM::Configuration n_wam_dof = barrett_direct::WAM_7DOF) :
      TaskContext(name),
      n_wam_dof_(n_wam_dof),
      can_dev_name_(can_dev_name),
      canbus_(can_dev_name_, CANBus::RATE_1000 ),
      robot_(&canbus_, n_wam_dof_),
      torques_(barrett_direct::WAM::DOF(n_wam_dof_)),
      positionss_(barrett_direct::WAM::DOF(n_wam_dof_))
    {
      std::cout << "WAM \""<<name<<"\" constructed !" <<std::endl;

      // Configure data ports
      this->ports()->addEventPort("torques_in", torques_in_port_).doc("Input Event port: nx1 vector of joint torques. (n joints)");
      this->ports()->addPort("positions_out", positions_out_port_).doc("Output port: nx1 vector of joint positions. (n joints)");

      // Prepare ports for realtime processing
      Eigen::VectorXd joints_sample(7);
      torqe_in_port_.setDataSample(joints_sample);
      positions_out_port_.setDataSample(joints_sample);
    }

    bool configureHook() {

      return true;
    }

    bool startHook() {
      // Check the data ports
      if ( !torqe_in_port_.connected() ) {
        std::cerr<<"ERROR: No connection to \"torques_in\" for WAM on \""<<can_dev_name_<<"\"!"<<std::endl;
        return false;
      }
      if ( !positions_out_port_.connected() ) {
        std::cerr<<"WARNING: No connection to \"positions_out\" for WAM on \""<<can_dev_name_<<"\"!"<<std::endl;
      }

      // Open the canbus
      if( can->Open() != leoCAN::CANBus::ESUCCESS ){
        std::cerr<<"Failed to open CAN device \""<<can_dev_name_<<"\""<<std::endl;
        return false;
      }
      
      // Initialize the WAM robot
      if( wam_robot->Initialize() != barrett_direct::WAM::ESUCCESS ){
        std::cerr<<"Failed to initialize WAM"<<std::endl;
        return false;
      }

      std::cout << "WAM started on CAN device \""<<can_dev_name_<<"\"!" <<std::endl;
      return true;
    }

    void updateHook() {
      // Get inputs

      // Only send joint torques if new data is coming in
      if( torques_in_port_.read( torques_ ) == RTT::NewData ) {
        robot_.SetTorques( torques_ ) ;
      }

      // Get joint positions
      robot_.GetPositions( position_ ) ;
      // Send 
    }

    void stopHook() {
    }

    void cleanupHook() {
    }

  private:

    // Configuration properties
    std::string can_dev_name_;
    barrett_direct::WAM::Configuration n_wam_dof_;

    // Hardware hooks
    leoCAN::RTSocketCAN canbus_;
    barrett_direct::WAM robot_;

    // Working variables
    Eigen::VectorXd torques_;
    Eigen::VevtorXd positions_;
  };
}


#endif // ifndef __BARD_COMPONENTS_WAM_H
