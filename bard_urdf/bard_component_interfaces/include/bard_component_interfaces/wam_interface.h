/*
 * Copyright (c) 2012, The Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The Johns Hopkins University. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BARD_HARDWARE_WAM_INTERFACE_H
#define __BARD_HARDWARE_WAM_INTERFACE_H

#include <iostream>

#include <kdl/jntarray.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <sensor_msgs/JointState.h>

#include <bard_common/util.h>

namespace bard_component_interfaces {
  class WAMInterface : public RTT::TaskContext
  {
  public:
    WAMInterface(std::string const& name) :
      TaskContext(name, RTT::base::TaskCore::PreOperational)
      // RTT Properties
      ,robot_description_("")
      ,root_link_("")
      ,tip_link_("")
      ,initial_positions_(7,0.0)
      ,joint_state_throttle_period_(0.01)
      // Other members
      ,n_dof_(0)
      ,torques_()
      ,positions_()
      ,positions_new_()
      ,joint_state_()
      ,joint_state_throttle_(joint_state_throttle_period_)
    {
    }

  protected:
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    std::vector<double> initial_positions_;
    RTT::os::TimeService::Seconds joint_state_throttle_period_;
    
    // RTT Ports
    RTT::InputPort<KDL::JntArray> torques_in_port_;
    RTT::OutputPort<KDL::JntArrayVel> positions_out_port_;
    RTT::OutputPort<sensor_msgs::JointState> joint_state_out_port_;

    // See: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
    // See: http://www.orocos.org/forum/orocos/orocos-users/some-info-eigen-and-orocos
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // RTT Operations
    virtual void calibrate_position(std::vector<double> &actual_positions) = 0; 
    virtual void set_velocity_warn(unsigned int thresh) = 0;
    virtual void set_velocity_fault(unsigned int thresh) = 0;
    virtual void set_torque_warn(unsigned int thresh) = 0;
    virtual void set_torque_fault(unsigned int thresh) = 0; 

    double get_loop_rate()
    {
      return 1.0/loop_period_;
    }

    void print_time()
    {
      RTT::os::TimeService *rtt_ts = RTT::os::TimeService::Instance();
      
      ROS_INFO_STREAM("TIME DIFFERENCE (ROS-RTT) RTT::os::TimeService:               "<<(ros::Time::now() - ros::Time(((double)rtt_ts->getNSecs())*1E-9)));
      ROS_INFO_STREAM("TIME DIFFERENCE (ROS-RTT) clock_gettime(CLOCK_HOST_REALTIME): "<<ros::Time::now() - bard_common::util::ros_rt_now());
    }

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    urdf::Model urdf_model_;
    std::vector<double> torque_limits_;

    // Dataport message temporary storage
    KDL::JntArray torques_;
    KDL::JntArrayVel positions_;
    KDL::JntArrayVel positions_new_;
    sensor_msgs::JointState joint_state_;

    // Performance metrics
    RTT::os::TimeService::ticks last_loop_time_;
    RTT::os::TimeService::Seconds loop_period_;

    // Other state
    bard_common::util::PeriodicThrottle joint_state_throttle_;

    // Common initialization code
    virtual void init_rtt_interface() 
    {
      // Declare properties (configuration variables)
      this->addProperty("robot_description",robot_description_)
        .doc("The WAM URDF xml string.");
      this->addProperty("initial_positions",initial_positions_)
        .doc("The calibration position of the robot.");
      this->addProperty("root_link",root_link_)
        .doc("The root link for the controller.");
      this->addProperty("tip_link",tip_link_)
        .doc("The tip link for the controller.");
      this->addProperty("joint_state_throttle_period",joint_state_throttle_period_)
        .doc("The period of the ROS sensor_msgs/JointState publisher.");

      // Configure data ports
      this->ports()->addPort("torques_in", torques_in_port_)
        .doc("Input Event port: nx1 vector of joint torques. (n joints)");
      this->ports()->addPort("positions_out", positions_out_port_)
        .doc("Output port: nx1 vector of joint positions & velocities. (n joints)");
      this->ports()->addPort("joint_state_out", joint_state_out_port_)
        .doc("Output port: sensor_msgs::JointState.");

      // Add operation for setting the encoder values
      this->provides("calibration")
        ->addOperation("calibrate_position", 
            &WAMInterface::calibrate_position, this, RTT::OwnThread)
        .doc("Set the angles that the encoders should read with the arm in the current configuration. This is used for calibrating the robot.")
        .arg("angles","The new joint angles.");

      // Add operations for setting warnings and faults
      this->addOperation("setVelocityWarning",
          &WAMInterface::set_velocity_warn, this, RTT::OwnThread)
        .doc("Set the velocities above which the WAMInterface pendant will illumiate a warning light.")
        .arg("thresh","Velocity Warning Threshold");
      this->addOperation("setVelocityFault",
          &WAMInterface::set_velocity_fault, this, RTT::OwnThread)
        .doc("Set the velocities above which the WAMInterface pendant will abruptly shut down the arm and illumiate a fault light.")
        .arg("thresh","Velocity Fault Threshold");
      this->addOperation("setTorqueWarning",
          &WAMInterface::set_torque_warn, this, RTT::OwnThread)
        .doc("Set the torques above which the WAMInterface pendant will illumiate a warning light.")
        .arg("thresh","Torque Warning Threshold");
      this->addOperation("setTorqueFault",
          &WAMInterface::set_torque_fault, this, RTT::OwnThread)
        .doc("Set the torques above which the WAMInterface pendant will abruptly shut down the arm and illumiate a fault light.")
        .arg("thresh","Torque Fault Threshold");

      this->addOperation("getLoopRate",
          &WAMInterface::get_loop_rate, this, RTT::OwnThread)
        .doc("Get the loop rate (Hz)");
      this->addOperation("printTime",
          &WAMInterface::print_time, this, RTT::OwnThread)
        .doc("Print the ROS and RTT time.");
    }

    virtual bool init_kinematics()
    {
      using namespace bard_common;

      // Initialize kinematics (KDL tree, KDL chain, and #DOF)
      if(!util::initialize_kinematics_from_urdf(
            robot_description_, root_link_, tip_link_,
            n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
      {
        ROS_ERROR("Could not initialize robot kinematics!");
        return false;
      }

      // Get torque limits from urdf
      torque_limits_.clear();
      for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
          it != kdl_chain_.segments.end();
          it++)
      {
        torque_limits_.push_back(urdf_model_.getJoint(it->getJoint().getName())->limits->effort);
      }

      // Resize joint arrays
      torques_ = KDL::JntArray(n_dof_);
      positions_ = KDL::JntArrayVel(n_dof_);
      positions_new_ = KDL::JntArrayVel(n_dof_);

      // Zero out joint arrays
      KDL::SetToZero(torques_);
      KDL::SetToZero(positions_.q); KDL::SetToZero(positions_.qdot);
      KDL::SetToZero(positions_new_.q); KDL::SetToZero(positions_new_.qdot);

      // Construct ros JointState message with the appropriate joint names
      util::joint_state_from_kdl_chain(kdl_chain_, joint_state_);

      // Prepare ports for realtime processing
      positions_out_port_.setDataSample(positions_);
      joint_state_out_port_.setDataSample(joint_state_);

      return true;
    }

    virtual void publish_throttled_joint_state()
    {
      // Copy joint positions into joint state
      if( joint_state_throttle_.ready(joint_state_throttle_period_)) {
        joint_state_.header.stamp = ros::Time::now();
        for(unsigned int i=0; i<n_dof_; i++) {
          joint_state_.position[i] = positions_.q(i);
          joint_state_.velocity[i] = positions_.qdot(i);
          joint_state_.effort[i] = torques_(i);
        }
        joint_state_out_port_.write( joint_state_ );
      } 
    }
  };
}


#endif // ifndef __BARD_HARDWARE_WAM_INTERFACE_H
