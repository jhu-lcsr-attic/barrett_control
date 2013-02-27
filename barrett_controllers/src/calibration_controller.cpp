///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Wim Meeussen
 */


#include <barrett_controllers/calibration_controller.h>
#include <barrett_control_msgs/SemiAbsoluteCalibrationState.h>
#include <pluginlib/class_list_macros.h>

#include <terse_roscpp/params.h>

#include <list>


namespace barrett_controllers
{

  CalibrationController::CalibrationController()
    : calibration_step_(IDLE), command_()
  {

  }

  CalibrationController::~CalibrationController()
  {
    command_sub_.shutdown();
  }

  void load_params() {

  }

  bool CalibrationController::init(
      barrett_model::SemiAbsoluteJointInterface* hw, 
      ros::NodeHandle &nh)
  {
    using namespace terse_roscpp;

    // get all joint states from the hardware interface
    const std::vector<std::string>& available_joint_names = hw->getJointNames();
    for (unsigned i=0; i<available_joint_names.size(); i++)
      ROS_DEBUG("Got joint %s", available_joint_names[i].c_str());

    // Get the list of joints to be calibrated
    require_param(nh, "joint_names", joint_names_,
        "The list of joints to be calibrated.");
    require_param(nh, "static_thresholds", static_thresholds_,
        "The position change threshold to determine if a joint is stationary (has reached a limit).");
    require_param(nh, "upper_limits", upper_limits_,
        "The upper limits of the joints.");
    require_param(nh, "lower_limits", lower_limits_,
        "The lower limits of the joints.");
    require_param(nh, "limit_search_directions", limit_search_directions_,
        "The direction to move to reach the limit of a given joint.");
    require_param(nh, "home_positions", home_positions_,
        "The positions that the joints should be in when they're calibrated.");
    require_param(nh, "resolver_offsets", resolver_offsets_,
        "The absolute resolver angles at the home positions.");

    require_param(nh, "p_gains", p_gains_, "PID Proportial gains.");
    require_param(nh, "i_gains", i_gains_, "PID Integral gains.");
    require_param(nh, "d_gains", d_gains_, "PID Derivative gains.");
    require_param(nh, "i_max", i_max_, "PID Integral gain bounds.");

    require_param(nh, "trap_max_vels", trap_max_vels_);
    require_param(nh, "trap_max_accs", trap_max_accs_);
    require_param(nh, "trap_durations", trap_durations_);

    // get publishing period
    if (!nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR_STREAM("Parameter 'publish_rate' in namespace "<<nh.getNamespace()<<" not set!");
      return false;
    }

    // realtime publisher
    realtime_pub_.reset(
        new realtime_tools::RealtimePublisher<barrett_control_msgs::SemiAbsoluteCalibrationState>(
          nh, "joint_calibration_state", 4));

    // get joints and allocate message
    joint_handles_.resize(joint_names_.size());
    command_.resize(joint_names_.size());
    calibration_states_.assign(joint_names_.size(),UNCALIBRATED);
    position_history_.assign(joint_names_.size(),std::list<double>());
    bomb_armed_.assign(joint_names_.size(),false);
    pids_.resize(joint_names_.size());
    trajectories_.resize(joint_names_.size());
    trajectory_start_times_.resize(joint_names_.size());
    position_errors_.resize(joint_names_.size());
    velocity_errors_.resize(joint_names_.size());
    for (unsigned i=0; i<joint_names_.size(); i++){
      pids_[i] = control_toolbox::Pid(p_gains_[i], i_gains_[i], d_gains_[i], i_max_[i], -i_max_[i]);
      trajectories_[i] = KDL::VelocityProfile_Trap(trap_max_vels_[i], trap_max_accs_[i]);
      joint_handles_[i] = hw->getSemiAbsoluteJointHandle(joint_names_[i]);
      realtime_pub_->msg_.name.push_back(joint_names_[i]);
      realtime_pub_->msg_.effort.push_back(0.0);
      realtime_pub_->msg_.position_errors.push_back(0.0);
      realtime_pub_->msg_.velocity_errors.push_back(0.0);
      realtime_pub_->msg_.resolver_angle.push_back(0.0);
      realtime_pub_->msg_.calibration_state.push_back(UNCALIBRATED);
    }    

    // ROS API
    command_sub_ = nh.subscribe("command", 1, &CalibrationController::command_cb, this);
    calibrate_srv_ = nh.advertiseService("calibrate", &CalibrationController::calibrate_srv_cb, this);

    return true;
  }

  void CalibrationController::starting(const ros::Time& time)
  {
    // Initialize time
    last_publish_time_ = time;
    // Zero the command
    command_.assign(command_.size(), 0.0);
    position_errors_.assign(command_.size(), 0.0);
    velocity_errors_.assign(command_.size(), 0.0);
  }


  void CalibrationController::update(const ros::Time& time, const ros::Duration& period)
  {
    for(unsigned jid=0; jid < joint_handles_.size(); jid++) {
      barrett_model::SemiAbsoluteJointHandle &joint = joint_handles_[jid];

      switch(calibration_states_[jid]) {
        case UNCALIBRATED:
          //TODO: hold fixed
          break;
        case CALIBRATED:
          // Hold fixed in home position
          command_[jid] = 
            pids_[jid].computeCommand(
                home_positions_[jid] - (joint.getOffset() + joint.getPosition()),
                0.0 - joint.getVelocity(),
                period);
          break;
        case CALIBRATING:
          // All actively calibrating joints are in the same step at a time
          switch(calibration_step_) {
            case IDLE:
              { break; }
            case START_LIMIT_SEARCH: 
              {
                // Reset offset
                joint.setOffset(0.0);
                // Create the trajectory
                // Relative move to limit
                if(limit_search_directions_[jid] > 0.0) {
                  trajectories_[jid].SetProfile(joint.getPosition(), joint.getPosition() + upper_limits_[jid]-lower_limits_[jid]);
                } else {
                  trajectories_[jid].SetProfile(joint.getPosition(), joint.getPosition() + lower_limits_[jid]-upper_limits_[jid]);
                }

                // Clear the position buffer
                position_history_[jid].clear();
                bomb_armed_[jid] = false;
                trajectory_start_times_[jid] = time;
                pids_[jid].reset();
              }
            case LIMIT_SEARCH: 
              {
                // Find the positive or negative limit of this joint
                // Drive towards the limit
                position_errors_[jid] = trajectories_[jid].Pos((time - trajectory_start_times_[jid]).toSec()) - joint.getPosition();
                velocity_errors_[jid] = trajectories_[jid].Vel((time - trajectory_start_times_[jid]).toSec()) - joint.getVelocity();
                command_[jid] = 
                  pids_[jid].computeCommand(
                      position_errors_[jid], velocity_errors_[jid], period);
                break;
              }
            case START_APPROACH_CALIB_REGION:
              {
                // Store the offset to get the approximate position
                // Create the trajectory
                if(limit_search_directions_[jid] > 0.0) {
                  joint.setOffset(upper_limits_[jid] - joint.getPosition());
                  trajectories_[jid].SetProfile(upper_limits_[jid], home_positions_[jid]);
                } else {
                  joint.setOffset(lower_limits_[jid] - joint.getPosition());
                  trajectories_[jid].SetProfile(lower_limits_[jid], home_positions_[jid]);
                }
                // Clear the position buffer
                position_history_[jid].clear();
                bomb_armed_[jid] = false;
                trajectory_start_times_[jid] = time;
                //NOTE: Don't reset gains here to create a smooth transition
                //between these two states
                break;
              }
            case APPROACH_CALIB_REGION: 
              {
                position_errors_[jid] = trajectories_[jid].Pos((time - trajectory_start_times_[jid]).toSec()) - (joint.getOffset() + joint.getPosition());
                velocity_errors_[jid] = trajectories_[jid].Vel((time - trajectory_start_times_[jid]).toSec()) - joint.getVelocity();

                command_[jid] = 
                  pids_[jid].computeCommand(
                      position_errors_[jid], velocity_errors_[jid], period);
                break;
              }
            case START_GO_HOME:
              {
                // Set the exact offset
                // TODO: deal with differential
                joint.setOffset(joint.getOffset() 
                    + joint.getShortestDistance(resolver_offsets_[jid],joint.getResolverAngle()));
                // Create the trajectory
                trajectories_[jid].SetProfile(joint.getOffset() + joint.getPosition(), home_positions_[jid]);
                // Clear the position buffer
                position_history_[jid].clear();
                bomb_armed_[jid] = false;
                trajectory_start_times_[jid] = time;
                pids_[jid].reset();
                break;
              }
            case GO_HOME: 
              {
                position_errors_[jid] = trajectories_[jid].Pos((time - trajectory_start_times_[jid]).toSec()) - (joint.getOffset() + joint.getPosition());
                velocity_errors_[jid] = trajectories_[jid].Vel((time - trajectory_start_times_[jid]).toSec()) - joint.getVelocity();

                command_[jid] = 
                  pids_[jid].computeCommand(
                      position_errors_[jid], velocity_errors_[jid], period);
                break;
              }
            case END_CALIBRATION: 
              {
                // Mark this joint as calibrated
                calibration_states_[jid] = CALIBRATED; 
                joint_handles_[jid].setCalibrated(true);
                pids_[jid].reset();
                break;
              }
          };
          break;
      };

      // Set the actual commands
      joint_handles_[jid].setCommand(command_[jid]);
    }

    // Flag to determine if all calibrating joints are static
    bool all_joints_static = true;
    for(unsigned jid=0; jid < joint_handles_.size(); jid++) {
      bool s = is_static(jid, joint_handles_[jid].getPosition());

      if(calibration_states_[jid] == CALIBRATING) {
        all_joints_static = all_joints_static && s;
        if(calibration_step_ != LIMIT_SEARCH) {
          all_joints_static = all_joints_static && fabs(position_errors_[jid]) < 0.05;
        }
      }
    }

    switch(calibration_step_) {
      case IDLE:
        // No-op
        break;
      case START_LIMIT_SEARCH: 
        calibration_step_ = LIMIT_SEARCH; break;
      case LIMIT_SEARCH:
        calibration_step_ = all_joints_static ? START_APPROACH_CALIB_REGION : LIMIT_SEARCH; break;

      case START_APPROACH_CALIB_REGION:
        calibration_step_ = APPROACH_CALIB_REGION; break;
      case APPROACH_CALIB_REGION:
        calibration_step_ = all_joints_static ? START_GO_HOME : APPROACH_CALIB_REGION; break;

      case START_GO_HOME:
        calibration_step_ = GO_HOME; break;
      case GO_HOME:
        calibration_step_ = all_joints_static ? END_CALIBRATION : GO_HOME; break;

      case END_CALIBRATION:
        calibration_step_ = IDLE;
        break;
    };


    // limit rate of publishing
    if (publish_rate_ > 0.0 
        && time - last_publish_time_ < ros::Duration(1.0/publish_rate_) )
    {
      // try to publish
      if (realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // populate joint state message
        realtime_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<joint_handles_.size(); i++){
          realtime_pub_->msg_.resolver_angle[i] = joint_handles_[i].getResolverAngle();
          realtime_pub_->msg_.effort[i] = command_[i];
          realtime_pub_->msg_.position_errors[i] = position_errors_[i];
          realtime_pub_->msg_.velocity_errors[i] = velocity_errors_[i];
          realtime_pub_->msg_.calibration_state[i] = calibration_states_[i];
        }
        realtime_pub_->msg_.calibration_step = calibration_step_;
        realtime_pub_->unlockAndPublish();
      }
    }
  }

  void CalibrationController::stopping(const ros::Time& time)
  {}

  bool CalibrationController::calibrate_srv_cb(
      barrett_control_msgs::Calibrate::Request &req,
      barrett_control_msgs::Calibrate::Response &resp)
  {
    if(calibration_step_ != IDLE) {
      ROS_ERROR("Please wait for the active joints to finish calibrating...");
      resp.ok = false;
    } else {
      for(std::vector<std::string>::iterator req_name = req.joint_names.begin();
          req_name != req.joint_names.end();
          ++req_name)
      {
        for(unsigned int i=0; i<joint_handles_.size(); i++) {
          if(joint_handles_[i].getName() == *req_name) {
            calibration_states_[i] = CALIBRATING;
            resp.ok = true;
            break;
          }
        }
      }

      // Begin calibration
      calibration_step_ = START_LIMIT_SEARCH;
    }

    return resp.ok;
  }


  void CalibrationController::command_cb(const barrett_control_msgs::JointEffortCommandConstPtr & msg)
  {
    for(unsigned int i=0; i<command_.size() && i <msg->effort.size(); i++) {
      command_[i] = msg->effort[i];
    }
  }
}


PLUGINLIB_DECLARE_CLASS(
    barrett_controllers,
    CalibrationController,
    barrett_controllers::CalibrationController,
    controller_interface::ControllerBase)

