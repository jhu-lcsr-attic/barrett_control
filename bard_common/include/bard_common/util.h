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

#ifndef __BARD_COMMON_UTIL_H
#define __BARD_COMMON_UTIL_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <sensor_msgs/JointState.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include <rtt/RTT.hpp>

#include <ros/ros.h>

namespace bard_common {
  namespace util {

    bool load_rosparam_and_refresh(RTT::TaskContext *tc);
    
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

    // Function to get a ros::Time initialized from RTT::os::TimeService
    ros::Time ros_rt_now();

    // Class for picking either ROS time or RT time
    class TimeLord {
    public:
      typedef enum {
        UNKNOWN = 0,
        WALL_TIME,
        SIM_TIME
      } time_state_t;

      static void Init() {
        if( TimeState_ == UNKNOWN ) {
          // Check if ROS is using simulation time
          ros::NodeHandle nh;
          bool use_sim_time = false;

          use_sim_time = nh.getParam("use_sim_time",use_sim_time) && use_sim_time;

          if(use_sim_time) {
            TimeState_ = SIM_TIME;
          } else {
            TimeState_ = WALL_TIME;
          }
        }
      }

      static ros::Time autotime_now() {
        TimeLord::Init();
        
        if(TimeState_ == SIM_TIME) {
          // Use the ROS clock time, which in this case will be the time from the /clock topic
          return ros::Time::now();
        } else {
          // Use NTP-synchronized RTC time in an RT system, and the ROS clock time in a non-rt system
          return util::ros_rt_now();
        }
      }

      static ros::Time rostime_now() {
        return ros::Time::now();
      }

      static ros::Time walltime_now() {
        return ros_rt_now();
      }

    protected:
      static time_state_t TimeState_;
    };

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

#endif // ifndef __BARD_COMMON_UTIL_H
