#ifndef __BARD_COMPONENTS_UTIL_H
#define __BARD_COMPONENTS_UTIL_H

#include <iostream>

#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include <rtt/RTT.hpp>

namespace bard_components {
  namespace util {
    
    // Function to create some KDL structures and get the #DOF from an URDF
    bool initialize_kinematics_from_urdf(
        const std::string &robot_description,
        const std::string &root_link,
        const std::string &tip_link,
        unsigned int &n_dof,
        KDL::Chain &kdl_chain,
        KDL::Tree &kdl_tree,
        urdf::Model &urdf_model);

    // Function to construct a sensor_msgs::JointState message from a kdl chain
    void joint_state_from_kdl_chain(
        const KDL::Chain &chain,
        sensor_msgs::JointState &joint_state);

    // Classes to throttle the rate at which something is being done in an RTT thread
    class PeriodicThrottle {
    public: 
      PeriodicThrottle(RTT::os::TimeService::Seconds throttle_period) : 
        throttle_period_(throttle_period),
        last_time_(0)
      { }

      inline bool ready(double throttle_period) {
        // Check timer
        if( throttle_period_ > 0.0
            && RTT::os::TimeService::Instance()->secondsSince(last_time_) > throttle_period  )
        {
          // Store this time
          last_time_ = RTT::os::TimeService::Instance()->getTicks();
          return true;
        } 
        return false;
      }

    private:
      RTT::os::TimeService::Seconds throttle_period_;
      RTT::os::TimeService::ticks last_time_;
    };

    class CounterThrottle {
    public: 
      CounterThrottle(size_t throttle_divider) : 
        throttle_divider_(throttle_divider),
        loop_count_(0)
      { }

      inline bool ready(size_t throttle_divider) {
        // Check counter
        if( throttle_divider_ > 0 && loop_count_ > throttle_divider) {
          loop_count_ = 0;
          return true;
        }
        // Increment counter
        loop_count_++;
        return false;
      }

    private:
      size_t throttle_divider_;
      size_t loop_count_;
    };

  }
}

#endif // ifndef __BARD_COMPONENTS_UTIL_H
