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

#ifndef __BARRETT_MODEL_WAM_INTERFACE_H
#define __BARRETT_MODEL_WAM_INTERFACE_H

#include <iostream>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <sensor_msgs/JointState.h>

#include <kdl_urdf_tools/tools.h>

#include <terse_roscpp/params.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


namespace barrett_model {
  class WAMInterface : public hardware_interface::RobotHW
  {
  public:

    enum {
      IDLE,
      CONFIGURED,
      CALIBRATED,
      STARTED
    };

    WAMInterface(ros::NodeHandle nh) :
      hardware_interface::RobotHW()
      ,nh_(nh)
      // State
      ,run_state_(IDLE)
      // Parameters
      ,robot_description_("")
      ,root_link_("")
      ,tip_link_("")
      ,initial_positions_(7,0.0)
      // Working variables
      ,n_dof_(0)
      ,torques_()
      ,joint_state_()
      ,joint_state_new_()
    {
    }

  protected:
    // Configuration
    ros::NodeHandle nh_;
    int run_state_;

    // Parameters
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    std::vector<double> initial_positions_;

    // Services
    virtual bool calibrate_position(std::vector<double> &actual_positions) = 0; 
    virtual void set_velocity_warn(unsigned int thresh) = 0;
    virtual void set_velocity_fault(unsigned int thresh) = 0;
    virtual void set_torque_warn(unsigned int thresh) = 0;
    virtual void set_torque_fault(unsigned int thresh) = 0; 

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    urdf::Model urdf_model_;
    std::vector<std::string> joint_names_;
    std::vector<double> torque_limits_;

    // Dataport message temporary storage
    KDL::JntArray torques_;
    KDL::JntArrayVel joint_state_;
    KDL::JntArrayVel joint_state_new_;

    // ROS Control hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::EffortJointInterface effort_command_interface_;

    // Calibration
    barrett_model::SemiAbsoluteJointInterface semi_absolute_interface_;
    KDL::JntArray motor_angles_;

    // Common initialization code
    virtual void load_params()
    {
      using namespace terse_roscpp;

      require_param(nh_,"robot_description",robot_description_,
                    "The WAM URDF xml string.");
      require_param(nh_,"initial_positions",initial_positions_,
                    "The calibration position of the robot.");
      require_param(nh_,"root_link",root_link_,
                    "The root link for the controller.");
      require_param(nh_,"tip_link",tip_link_,
                    "The tip link for the controller.");

    }

    virtual bool init_kinematics()
    {
      // Initialize kinematics (KDL tree, KDL chain, and #DOF)
      if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
              robot_description_, root_link_, tip_link_,
              n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
      {
        ROS_ERROR("Could not initialize robot kinematics!");
        return false;
      }

      // Get torque limits from urdf
      torque_limits_.clear();
      for(std::vector<KDL::Segment>::const_iterator segment=kdl_chain_.segments.begin();
          segment != kdl_chain_.segments.end();
          segment++)
      {
        joint_names_.push_back(segment->getJoint().getName());
        torque_limits_.push_back(urdf_model_.getJoint(joint_names_.back())->limits->effort);
      }

      // Resize joint arrays
      torques_.resize(n_dof_);
      joint_state_.resize(n_dof_);
      joint_state_new_.resize(n_dof_);
      motor_angles_.resize(n_dof_);

      // Zero out joint arrays
      // TODO: why can't we call KDL::SetToZero(joint_state_)?
      KDL::SetToZero(torques_);
      KDL::SetToZero(joint_state_.q);
      KDL::SetToZero(joint_state_.qdot);
      KDL::SetToZero(joint_state_new_.q);
      KDL::SetToZero(joint_state_new_.qdot);
      KDL::SetToZero(motor_angles_);

      return true;
    }

    virtual bool register_hardware_interfaces()
    {
      // Register the joints
      for(unsigned int j=0; j < n_dof_; j++) {
        // Register this joint with the joint state interface
        joint_state_interface_.registerJoint(
            joint_names_[j], 
            &joint_state_.q(j),
            &joint_state_.qdot(j), 
            &torques_(j));

        // Register this joint with the effort command interface
        effort_command_interface_.registerJoint(
            joint_state_interface_.getJointStateHandle(joint_names_[j]),
            &torques_(j));

        semi_absolute_interface_.registerJoint(
            effort_command_interface_.getSemiAbsoluteJointHandle(joint_names_[j]),
            &motor_angles_(j));
      }

      // Register interfaces
      registerInterface(&joint_state_interface_);
      registerInterface(&effort_command_interface_);
      registerInterface(&semi_absolute_interface_);

      return true;
    }

    bool is_configured() 
    {
      return run_state_ == CONFIGURED || run_state_ == STARTED;
    }
  };
}


#endif // ifndef __BARRETT_MODEL_WAM_INTERFACE_H
