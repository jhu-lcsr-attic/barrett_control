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

#ifndef __BARD_CONTROLLERS_CONTROLLERS_JOINT_PID
#define __BARD_CONTROLLERS_CONTROLLERS_JOINT_PID

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <sensor_msgs/JointState.h>

namespace bard_controllers {
  namespace controllers {
    class JointPID : public RTT::TaskContext
    {
      // RTT Properties
      std::string robot_description_;
      std::string root_link_;
      std::string tip_link_;
      std::vector<double> kp_, ki_, i_clamp_, kd_;
      RTT::os::TimeService::Seconds joint_state_throttle_period_;

      // RTT Ports
      RTT::InputPort<KDL::JntArrayVel> positions_in_port_;
      RTT::InputPort<KDL::JntArrayVel> positions_des_in_port_;
      RTT::OutputPort<KDL::JntArray> torques_out_port_;
      RTT::OutputPort<sensor_msgs::JointState> joint_state_out_port_;

    public:
      JointPID(std::string const& name);
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      bool within_error(KDL::JntArray &pos_des);

    private:

      // Working variables
      unsigned int n_dof_;
      KDL::Chain kdl_chain_;
      KDL::Tree kdl_tree_;
      urdf::Model urdf_model_;

      KDL::JntArrayVel positions_;
      KDL::JntArrayVel positions_des_;
      KDL::JntArray p_error_;
      KDL::JntArray i_error_;
      KDL::JntArray d_error_;
      KDL::JntArray torques_;

      sensor_msgs::JointState joint_state_;
      bard_common::util::PeriodicThrottle joint_state_throttle_;
    };
  }
}


#endif // ifndef __BARD_CONTROLLERS_CONTROLLERS_JOINT_PID

