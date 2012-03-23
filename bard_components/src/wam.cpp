#include <iostream>

#include <Eigen/Dense>

#include <ros/ros.h>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <sensor_msgs/JointState.h>

#include <bard_components/util.h>
#include <bard_components/wam.h>

using namespace bard_components;

WAM::WAM(string const& name) :
  TaskContext(name, RTT::base::TaskCore::PreOperational)
  // Properties
  ,can_dev_name_("")
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,initial_positions_(7,0.0)
  ,joint_state_throttle_period_(0.01)
  // Internal variables
  ,canbus_(NULL)
  ,robot_(NULL)
  ,needs_calibration_(true)
  ,n_dof_(0)
  ,torques_()
  ,positions_()
  ,positions_new_()
  ,joint_state_()
  ,joint_state_throttle_(joint_state_throttle_period_)
{
  // Declare properties (configuration variables)
  this->addProperty("can_dev_name",can_dev_name_)
     .doc("The name of the RTCAN device to which this WAM robot is connected.");
  this->addProperty("robot_description",robot_description_)
     .doc("The WAM URDF xml string.");
  this->addProperty("initial_positions",initial_positions_)
     .doc("The calibration position of the robot.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("joint_state_throttle_period",joint_state_throttle_period_)
     .doc("The period of the ROS sensor_msgs/JointState publisher.");

  // Configure data ports
  this->ports()->addEventPort("torques_in", torques_in_port_)
   .doc("Input Event port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("positions_out", positions_out_port_)
   .doc("Output port: nx1 vector of joint positions & velocities. (n joints)");
  this->ports()->addPort("joint_state_out", joint_state_out_port_)
   .doc("Output port: sensor_msgs::JointState.");

  // Add operation for setting the encoder values
  this->provides("calibration")
    ->addOperation("calibrate_position", &WAM::calibrate_position, this, RTT::OwnThread)
    .doc("Set the angles that the encoders should read with the arm in the current configuration. This is used for calibrating the robot.")
    .arg("angles","The new joint angles.");

  // Add operations for setting warnings and faults
  this->addOperation("setVelocityWarning", &WAM::set_velocity_warn, this, RTT::OwnThread)
    .doc("Set the velocities above which the WAM pendant will illumiate a warning light.")
    .arg("thresh","Velocity Warning Threshold");
  this->addOperation("setVelocityFault", &WAM::set_velocity_fault, this, RTT::OwnThread)
    .doc("Set the velocities above which the WAM pendant will abruptly shut down the arm and illumiate a fault light.")
    .arg("thresh","Velocity Fault Threshold");
  this->addOperation("setTorqueWarning", &WAM::set_torque_warn, this, RTT::OwnThread)
    .doc("Set the torques above which the WAM pendant will illumiate a warning light.")
    .arg("thresh","Torque Warning Threshold");
  this->addOperation("setTorqueFault", &WAM::set_torque_fault, this, RTT::OwnThread)
    .doc("Set the torques above which the WAM pendant will abruptly shut down the arm and illumiate a fault light.")
    .arg("thresh","Torque Fault Threshold");

  this->addOperation("getLoopRate", &WAM::get_loop_rate, this, RTT::OwnThread)
    .doc("Get the loop rate (Hz)");

  ROS_INFO_STREAM("WAM \""<<name<<"\" constructed !");
}

bool WAM::configureHook()
{
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  if(!bard_components::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }

  // Resize joint arrays
  torques_ = KDL::JntArray(n_dof_);
  positions_ = KDL::JntArrayVel(n_dof_);
  positions_new_ = KDL::JntArrayVel(n_dof_);

  // Zero out joint arrays
  KDL::SetToZero(torques_);
  KDL::SetToZero(positions_.q); KDL::SetToZero(positions_.qdot);
  KDL::SetToZero(positions_new_.q); KDL::SetToZero(positions_new_.qdot);
  
  // Construct ros JointState message with the appropriate joint names
  bard_components::util::joint_state_from_kdl_chain(kdl_chain_, joint_state_);

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_);
  joint_state_out_port_.setDataSample(joint_state_);

  // Try to connect and initialize hardware
  try{
    // Construct CAN structure
    canbus_.reset(new leoCAN::RTSocketCAN(can_dev_name_, leoCAN::CANBus::RATE_1000 ));

    // Open the canbus
    if( canbus_->Open() != leoCAN::CANBus::ESUCCESS ){
      ROS_ERROR_STREAM("Failed to open CAN device \""<<can_dev_name_<<"\"");
      throw std::exception();
    }

    // Construct WAM structure
    robot_.reset(new barrett_direct::WAM(canbus_.get(), (barrett_direct::WAM::Configuration)n_dof_));

    // Initialize the WAM robot
    if( robot_->Initialize() != barrett_direct::WAM::ESUCCESS ){
      ROS_ERROR_STREAM("Failed to initialize WAM");
      throw std::exception();
    }
  } catch(std::exception &ex) {
    // Free the device handles
    this->cleanup_internal();
    return false;
  }

  ROS_INFO_STREAM("WAM connected on CAN device \""<<can_dev_name_<<"\"!");

  return true;
}

bool WAM::startHook()
{
  // Check the data ports
  if ( !torques_in_port_.connected() ) {
    ROS_WARN_STREAM("WARNING: No connection to \"torques_in\" for WAM on \""<<can_dev_name_<<"\"!");
  }
  if ( !positions_out_port_.connected() ) {
    ROS_WARN_STREAM("WARNING: No connection to \"positions_out\" for WAM on \""<<can_dev_name_<<"\"!");
  }

  if(needs_calibration_) {
    // Set the joints to the calibration position
    this->calibrate_position(initial_positions_);
    needs_calibration_ = false;
  }

  // Set the robot to Activated
  if( robot_->SetMode(barrett_direct::WAM::MODE_ACTIVATED) != barrett_direct::WAM::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to ACTIVATE WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }

  ROS_INFO_STREAM("WAM started on CAN device \""<<can_dev_name_<<"\"!");
  return true;
}

void WAM::updateHook()
{
  // Only send joint torques if new data is coming in
  if( torques_in_port_.read( torques_ ) == RTT::NewData ) {
    if( robot_->SetTorques( torques_.data ) != barrett_direct::WAM::ESUCCESS ) {
      ROS_ERROR_STREAM("Failed to set torques of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
    }
  }
  
  // Get joint positions
  if( robot_->GetPositions( positions_new_.q.data ) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Failed to get positions of WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }

  // Get the actual loop period
  loop_period_ = RTT::os::TimeService::Instance()->secondsSince(last_loop_time_);
  // Compute joint velocities
  for(unsigned int i=0; i<n_dof_; i++) {
    positions_.qdot(i) = (positions_new_.q(i) - positions_.q(i))/loop_period_;
  }
  last_loop_time_ = RTT::os::TimeService::Instance()->getTicks();

  // Update positions
  positions_.q = positions_new_.q;

  // Send joint positions
  positions_out_port_.write( positions_ );

  // Copy joint positions into joint state
  if( joint_state_throttle_.ready(joint_state_throttle_period_)) {
    joint_state_.header.stamp = ros::Time::now();
    for(unsigned int i=0; i<n_dof_; i++) {
      joint_state_.position[i] = positions_.q(i);
      joint_state_.velocity[i] = positions_.qdot(i);
      joint_state_.effort[i] = torques_(i);
    }
    joint_state_out_port_.write( joint_state_ );
  } 
}

void WAM::stopHook()
{
  // Set the robot to IDLE
  if( robot_->SetMode(barrett_direct::WAM::MODE_IDLE) != barrett_direct::WAM::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to IDLE WAM Robot on CAN device \""<<can_dev_name_<<"\"");
  }
}

void WAM::cleanupHook()
{
  // Close the CANBus
  if( canbus_->Close() != leoCAN::CANBus::ESUCCESS ){
    ROS_ERROR_STREAM("Failed to close CAN device \""<<can_dev_name_<<"\"");
  }

  // Reset calibration flag
  needs_calibration_ = true;

  // Free the device handles
  this->cleanup_internal();
}

void WAM::calibrate_position(std::vector<double> &actual_positions)
{
  // Make sure we have a connection to the robot
  if(this->isConfigured()) {
    // Assign the positions to the current robot configuration
    if(robot_->SetPositions(Eigen::Map<Eigen::VectorXd>(&actual_positions[0],actual_positions.size()))
        != barrett_direct::WAM::ESUCCESS)
    {
      ROS_ERROR_STREAM("Failed to calibrate encoders!");
      return;
    }

    // Set the current positions to the initial positions
    for(size_t i=0; i<actual_positions.size(); i++) {
      positions_.q(i) = actual_positions[i];
    }

    ROS_INFO("Calibrated encoders.");
  } else {
    ROS_ERROR_STREAM("Cannot calibrate encoders! The WAM control task on device "<<can_dev_name_<<" is not configured.");
  }
}

void WAM::cleanup_internal()
{
  // Reset the scoped pointers
  robot_.reset(NULL);
  canbus_.reset(NULL);
}

void WAM::set_velocity_warn(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetVelocityWarning(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set velocity warning threshold.");
  }
}
void WAM::set_velocity_fault(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetVelocityFault(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set velocity fault threshold.");
  }
}
void WAM::set_torque_warn(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetTorqueWarning(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set torque warning threshold.");
  }
}
void WAM::set_torque_fault(unsigned int thresh)
{
  if(!this->isConfigured() || robot_->SetTorqueFault(thresh) != barrett_direct::WAM::ESUCCESS) {
    ROS_ERROR_STREAM("Could not set torque fault threshold.");
  }
}


double WAM::get_loop_rate() {
  return 1.0/loop_period_;
}
