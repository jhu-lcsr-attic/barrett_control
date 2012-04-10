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

#ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
#define __BARD_COMPONENTS_CONTROLLER_MUX_H

#include <iostream>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <bard_msgs/MuxState.h>
#include <bard_common/util.h>

#include <sensor_msgs/JointState.h>
#include <urdf/model.h>


// This class can be connected to multiple controllers, and performs
// automatic validation to ensure that large impulses do not arise from
// switching controllers at runtime.

namespace bard_controllers {
  class ControllerMux : public RTT::TaskContext {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    RTT::os::TimeService::Seconds joint_state_throttle_period_;

    // Structure to associte with each of the multiplexed controllers
    struct ControllerInterface {
      RTT::InputPort<KDL::JntArray> in_port;
      unsigned int dof;
      bool enabled;
      std::vector<int> dof_map;
      KDL::JntArray gains;
      KDL::JntArray last_torques;
    };
    
    // Vector of RTT input ports (torques)
    std::map<std::string, ControllerInterface*> controller_interfaces_;
    typedef std::map<std::string, ControllerInterface*>::iterator ControllerInterface_iter;

    // Input port for configuring the controller mux over ros
    RTT::InputPort<bard_msgs::MuxState> config_input_;
    RTT::InputPort<KDL::JntArrayVel> positions_in_port_;

    // Output torque
    RTT::OutputPort<KDL::JntArray> torques_out_port_;
    RTT::OutputPort<bard_msgs::MuxState> state_output_;
    RTT::OutputPort<sensor_msgs::JointState> joint_state_out_port_;

  public:
    ControllerMux(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook(); 
    void stopHook();
    void cleanupHook();

  private:

    // Global enable across all multiplexed controllers
    void enable();
    void disable();

    // Configuration of controllers
    void load_controller(std::string name);

    void unload_controller(std::string name);

    void toggle_controllers(
        std::vector<std::string> enable_controllers, 
        std::vector<std::string> diable_controllers);

    void list_controllers();

    // Working variables
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    urdf::Model urdf_model_;
    KDL::JntArray controller_torques_;
    KDL::JntArrayVel positions_;
    KDL::JntArray torques_;
    bool enabled_;

    bard_msgs::MuxState config_cmd_;
    sensor_msgs::JointState joint_state_;

    bard_common::util::PeriodicThrottle joint_state_throttle_;
  };
}

#endif // ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
