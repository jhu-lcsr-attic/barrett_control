#ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
#define __BARD_COMPONENTS_CONTROLLER_MUX_H

#include <iostream>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarray.hpp>

#include <bard_msgs/MuxState.h>

// This class can be connected to multiple controllers, and performs
// automatic validation to ensure that large impulses do not arise from
// switching controllers at runtime.

namespace bard_components {
  class ControllerMux : public RTT::TaskContext {
    // Properties
    std::string robot_description_;
    RTT::os::TimeService::Seconds joint_state_throttle_period_;

    // Structure to associte with each of the multiplexed controllers
    struct ControllerInterface {
      RTT::InputPort<KDL::JntArray> in_port;
      int dof;
      bool enabled;
      std::vector<int> dof_map;
      KDL::JntArray gains;
    };
    
    // Vector of RTT input ports (torques)
    std::map<std::string, ControllerInterface*> controller_interfaces_;
    typedef std::map<std::string, ControllerInterface*>::iterator ControllerInterface_iter;

    // Input port for controlling the controller mux over ros
    RTT::InputPort<bard_msgs::MuxState> config_input_;
    RTT::InputPort<KDL::JntArray> positions_in_port_;

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
    void load_controller(std::string name, int dof);

    void unload_controller(std::string name);

    void toggle_controllers(
        std::vector<std::string> enable_controllers, 
        std::vector<std::string> diable_controllers);

    // Working variables
    unsigned int n_dof_;
    KDL::JntArray controller_torques_;
    KDL::JntArray positions_;
    KDL::JntArray torques_;
    bool enabled_;

    bard_msgs::MuxState config_cmd_;
    sensor_msgs::JointState joint_state_;

    bard_components::util::PeriodicThrottle joint_state_throttle_;
  };
}

#endif // ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
