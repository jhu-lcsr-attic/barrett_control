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

namespace barrett_controllers {

  class CalibrationController : public controller_interface::Controller<barrett_model::SemiAbsoluteJointInterface>
  {
  public:
    virtual bool init(
        barrett_model::SemiAbsoluteJointInterface* hw,
        ros::NodeHandle &n);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping(const ros::Time& time);

  private:
    std::vector<barrett_model::SemiAbsoluteJointHandle> joint_handles_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<barrett_control_msgs::SemiAbsoluteCalibrationState> > 
      realtime_pub_;
    ros::Time last_publish_time_;
    double publish_rate_;
  };

}

#endif // ifndef __BARRETT_CONTROLLERS_CALIBRATION_CONTROLLER_H
