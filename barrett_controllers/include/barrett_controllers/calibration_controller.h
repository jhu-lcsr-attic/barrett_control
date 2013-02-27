#ifndef __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H
#define __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <barrett_model/semi_absolute_joint_interface.h>
#include <barrett_control_msgs/SemiAbsoluteCalibrationState.h>
#include <barrett_control_msgs/JointCommand.h>
#include <barrett_control_msgs/Calibrate.h>
#include <control_toolbox/pid.h>
#include <kdl/velocityprofile_trap.hpp>

namespace barrett_controllers {

  class CalibrationController : public controller_interface::Controller<barrett_model::SemiAbsoluteJointInterface>
  {
  public:

    typedef enum {
      IDLE = 0,
      START_LIMIT_SEARCH,
      LIMIT_SEARCH,
      START_APPROACH_CALIB_REGION,
      APPROACH_CALIB_REGION,
      START_GO_HOME,
      GO_HOME,
      END_CALIBRATION
    } calibration_step_t;

    typedef enum {
      UNCALIBRATED = 0,
      CALIBRATING = 1,
      CALIBRATED = 2
    } calibration_state_t;

    CalibrationController();
    ~CalibrationController();

    virtual bool init(
        barrett_model::SemiAbsoluteJointInterface* hw,
        ros::NodeHandle &n);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping(const ros::Time& time);

    bool calibrate_srv_cb(
        barrett_control_msgs::Calibrate::Request &req,
        barrett_control_msgs::Calibrate::Response &resp);

    void effort_command_cb(const barrett_control_msgs::JointCommandConstPtr & msg);
    void position_command_cb(const barrett_control_msgs::JointCommandConstPtr & msg);
  private:

    bool is_static(const int jid, const double position) {
      // We've reached the limit if the last 10 samples are within the static threshold
      double min_pos = *std::min_element(position_history_[jid].begin(), position_history_[jid].end());
      double max_pos = *std::max_element(position_history_[jid].begin(), position_history_[jid].end());

      position_history_[jid].push_back(position);
      while(position_history_[jid].size() > 100) {
        position_history_[jid].pop_front();
      }

      if(position_history_[jid].size() == 100) {
        if(max_pos - min_pos > static_thresholds_[jid]) {
          bomb_armed_[jid] = true;
          return false;
        } else if(bomb_armed_[jid] && max_pos - min_pos < static_thresholds_[jid]) {
          return true;
        } 
      }

      return false;
    }

    std::vector<barrett_model::SemiAbsoluteJointHandle> joint_handles_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<barrett_control_msgs::SemiAbsoluteCalibrationState> > 
      realtime_pub_;
    ros::Time last_publish_time_;
    double publish_rate_;

    std::vector<std::string> joint_names_;
    std::vector<double> 
      static_thresholds_,
      upper_limits_,
      lower_limits_,
      limit_search_directions_,
      home_positions_,
      resolver_offsets_,
      p_gains_,
      i_gains_,
      d_gains_,
      i_max_,
      trap_max_vels_,
      trap_max_accs_,
      trap_durations_,
      bomb_armed_,
      position_errors_,
      velocity_errors_;
    std::vector<calibration_state_t> calibration_states_;
    std::vector<control_toolbox::Pid> pids_;
    calibration_step_t calibration_step_;
    std::vector<bool> static_joints_;

    std::vector<std::list<double> > position_history_;
    std::vector<double> approximate_offsets_;
    std::vector<double> exact_offsets_;

    std::vector<KDL::VelocityProfile_Trap> trajectories_;
    std::vector<ros::Time> trajectory_start_times_;

    std::vector<int> active_joints_;

    std::vector<double> effort_command_;
    std::vector<double> position_command_;
    ros::Subscriber effort_command_sub_;
    ros::Subscriber position_command_sub_;
    ros::ServiceServer calibrate_srv_;
    bool auto_advance_;
  };

}

#endif // ifndef __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H
