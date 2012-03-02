#ifndef __BARD_COMPONENTS_WAM_H
#define __BARD_COMPONENTS_WAM_H

#include <Eigen/Dense>

#include <barrett_direct/WAM.h>
#include <leoCAN/RTSocketCAN.h>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <iostream>

namespace bard_components {
  class WAM : public RTT::TaskContext
  {
    // RTT Interface
    RTT::InputPort<Eigen::VectorXd> torques_in_port_;
    RTT::OutputPort<Eigen::VectorXd> positions_out_port_;

    // See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    // See: http://www.orocos.org/forum/orocos/orocos-users/some-info-eigen-and-orocos
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:
    WAM(string const& name) :
      TaskContext(name),
      n_wam_dof_(7),
      can_dev_name_("rtcan0"),
      canbus_(NULL),
      robot_(NULL),
      torques_(n_wam_dof_),
      positions_(n_wam_dof_)
    {
      // Declare properties (configuration variables)
      this->addProperty("can_dev_name",can_dev_name_).doc("The name of the RTCAN device to which this WAM robot is connected.");
      this->addProperty("n_wam_dof",n_wam_dof_).doc("The name of the RTCAN device to which this WAM robot is connected.");

      // Configure data ports
      this->ports()->addEventPort("torques_in", torques_in_port_).doc("Input Event port: nx1 vector of joint torques. (n joints)");
      this->ports()->addPort("positions_out", positions_out_port_).doc("Output port: nx1 vector of joint positions. (n joints)");

      // Prepare ports for realtime processing
      Eigen::VectorXd joints_sample(7);
      positions_out_port_.setDataSample(joints_sample);
      
      // Add operation for setting the encoder values
      this->addOperation("setEncoders", &barrett_direct::WAM::SetPositions, robot_, RTT::OwnThread)
        .doc("Set the values that the encoders should read. This is used for calibrating the robot.")
        .arg("Encoder Values","The new values for the encoders.");

      // Add operations for setting warnings and faults
      this->addOperation("setVelocityWarning", &barrett_direct::WAM::SetVelocityWarning, robot_, RTT::OwnThread)
        .doc("Set the velocities above which the WAM pendant will illumiate a warning light.")
        .arg("thresh","Velocity Warning Threshold");
      this->addOperation("setVelocityFault", &barrett_direct::WAM::SetVelocityFault, robot_, RTT::OwnThread)
        .doc("Set the velocities above which the WAM pendant will abruptly shut down the arm and illumiate a fault light.")
        .arg("thresh","Velocity Fault Threshold");
      this->addOperation("setTorqueWarning", &barrett_direct::WAM::SetTorqueWarning, robot_, RTT::OwnThread)
        .doc("Set the velocities above which the WAM pendant will illumiate a warning light.")
        .arg("thresh","Torque Warning Threshold");
      this->addOperation("setTorqueFault", &barrett_direct::WAM::SetTorqueFault, robot_, RTT::OwnThread)
        .doc("Set the velocities above which the WAM pendant will abruptly shut down the arm and illumiate a fault light.")
        .arg("thresh","Torque Fault Threshold");
      
      std::cout << "WAM \""<<name<<"\" constructed !" <<std::endl;
    }

    bool configureHook() {
      return true;
    }

    bool startHook() {
      // Check the data ports
      if ( !torques_in_port_.connected() ) {
        std::cerr<<"ERROR: No connection to \"torques_in\" for WAM on \""<<can_dev_name_<<"\"!"<<std::endl;
        return false;
      }
      if ( !positions_out_port_.connected() ) {
        std::cerr<<"WARNING: No connection to \"positions_out\" for WAM on \""<<can_dev_name_<<"\"!"<<std::endl;
      }
      
      // Reconstruct CAN and WAM structures
      canbus_ = new leoCAN::RTSocketCAN(can_dev_name_, leoCAN::CANBus::RATE_1000 );
      robot_ = new barrett_direct::WAM(canbus_, (barrett_direct::WAM::Configuration)n_wam_dof_);

      // Open the canbus
      if( canbus_->Open() != leoCAN::CANBus::ESUCCESS ){
        std::cerr<<"Failed to open CAN device \""<<can_dev_name_<<"\""<<std::endl;
        return false;
      }
      
      // Initialize the WAM robot
      if( robot_->Initialize() != barrett_direct::WAM::ESUCCESS ){
        std::cerr<<"Failed to initialize WAM"<<std::endl;
        return false;
      }

      std::cout << "WAM started on CAN device \""<<can_dev_name_<<"\"!" <<std::endl;
      return true;
    }

    void updateHook() {

      // Only send joint torques if new data is coming in
      if( torques_in_port_.read( torques_ ) == RTT::NewData ) {
        if( robot_->SetTorques( torques_ ) != barrett_direct::WAM::ESUCCESS ) {
          std::cerr<<"Failed to set torques of WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
        }
      }

      // Get joint positions
      if( robot_->GetPositions( positions_ ) != barrett_direct::WAM::ESUCCESS) {
          std::cerr<<"Failed to get positions of WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
      }

      // Send joint positions
      positions_out_port_.write( positions_ );
    }

    void stopHook() {
      // Set the robot to IDLE
      if( robot_->SetMode(0) != barrett_direct::WAM::ESUCCESS ){
        std::cerr<<"Failed to IDLE WAM Robot on CAN device \""<<can_dev_name_<<"\""<<std::endl;
      }
      // Close the CANBus
      if( canbus_->Close() != leoCAN::CANBus::ESUCCESS ){
        std::cerr<<"Failed to close CAN device \""<<can_dev_name_<<"\""<<std::endl;
      }
      if(robot_) {
        delete robot_;
        robot_ = NULL;
      }
      if(canbus_) {
        delete canbus_;
        canbus_ = NULL;
      }

    }

    void cleanupHook() {
    }

  private:

    // Configuration properties
    int n_wam_dof_;
    std::string can_dev_name_;

    // Hardware hooks
    leoCAN::RTSocketCAN *canbus_;
    barrett_direct::WAM *robot_;

    // Working variables
    Eigen::VectorXd torques_;
    Eigen::VectorXd positions_;

  };
}


#endif // ifndef __BARD_COMPONENTS_WAM_H
