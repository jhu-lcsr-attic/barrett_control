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

// This class can be connected to multiple controllers, and performs
// automatic validation to ensure that large impulses do not arise from
// switching controllers at runtime.

namespace bard_components {
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

    std::vector<double> torque_limits_;

    bard_msgs::MuxState config_cmd_;
    sensor_msgs::JointState joint_state_;

    bard_components::util::PeriodicThrottle joint_state_throttle_;
  };
}

#endif // ifndef __BARD_COMPONENTS_CONTROLLER_MUX_H
