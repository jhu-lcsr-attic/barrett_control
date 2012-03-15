#ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
#define __BARD_COMPONENTS_CONTROLLER_MUX_H

#include <iostream>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

// This class can be connected to multiple controllers, and performs
// automatic validation to ensure that large impulses do not arise from
// switching controllers at runtime.

namespace bard_components {
  class ControllerMux : public RTT::TaskContext {
    // Properties
    int n_arm_dof_;
    std::string joint_prefix_;
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
    RTT::OutputPort torque_out_port_;
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

    void unload_controller(std::string name, int dof);

    void toggle_controllers(
        std::vector<std::string> enable_controllers, 
        std::vector<std::string> diable_controllers);

    // Working variables
    KDL::JntArray controller_torques_;
    KDL::JntArray positions_;
    KDL::JntArray torques_;
    bool enabled_;

    sensor_msgs::JointState joint_state_;
    RTT::os::TimeService::ticks joint_state_pub_time_;

  };
}

#endif // ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
