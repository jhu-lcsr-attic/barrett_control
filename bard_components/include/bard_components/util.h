#ifndef __BARD_COMPONENTS_UTIL_H
#define __BARD_COMPONENTS_UTIL_H

#include <iostream>
#include <sensor_msgs/JointState.h>

namespace bard_components {
  namespace util {

    class Throttler {
      // A class to throttle things happening in an RT loop
    };

    // A function to initialize a ROS sensor_msgs/JointState message
    void init_wam_joint_state (
        const int n_wam_dof,
        const std::string &joint_prefix,
        sensor_msgs::JointState &joint_state);
  }
}

#endif // ifndef __BARD_COMPONENTS_UTIL_H
