#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <bard_components/util.h>
#include <bard_components/controllers/joint_trajectory.h>

using namespace bard_components::controllers;

JointTrajectory::JointTrajectory(string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,kdl_chain_()
  ,kdl_tree_()
  ,positions_()
  ,positions_des_()
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");

  // Configure data ports
  this->ports()->addEventPort("positions_in", positions_in_port_).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("trajectories_in", trajectories_in_port_).doc("Input port: nx1 vector of desired joint positions. (n joints)");
  this->ports()->addPort("positions_out", positions_out_port_).doc("Output port: nx1 vector of joint positions. (n joints)");
}

bool JointTrajectory::configureHook()
{
  // Construct an URDF model from the xml string
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  if(!bard_components::util::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model_))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
    return false;
  }

  // Resize working vectors
  positions_.resize(n_dof_);
  positions_des_.resize(n_dof_);

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_des_);

  return true;
}

bool JointTrajectory::startHook()
{
  return true;
}

// Comparison function for binary search
bool point_time_cmp(
    trajectory_msgs::JointTrajectoryPoint i,
    trajectory_msgs::JointTrajectoryPoint j)
{
  return (i.time_from_start < j.time_from_start);
}
      

void JointTrajectory::updateHook()
{
  // Read in the current joint positions & velocities
  positions_in_port_.readNewest( positions_ );

  // Read in the trajectories until there are none left to read
  RTT::FlowStatus trajectories_in_status = RTT::NoData;
  
  do{
    // Try to read in a trajectory 
    trajectories_in_status = trajectories_in_port_.read( new_trajectory_ );

    // Splice in this new trajectory if it's new and has any traj points
    if(trajectories_in_status == RTT::NewData && new_trajectory_.points.size() > 0)
    {
      // Duration representing the start time of this trajectory
      ros::Duration init_duration = ros::Duration(new_trajectory_.header.stamp.sec,  new_trajectory_.header.stamp.nsec);

      // Declare bounds for binary search
      std::pair<
        std::list<trajectory_msgs::JointTrajectoryPoint>::iterator,
        std::list<trajectory_msgs::JointTrajectoryPoint>::iterator> insertion_bounds;

      // Compute the time of the start of the new trajectory
      trajectory_msgs::JointTrajectoryPoint first_point = new_trajectory_.points[0];
      first_point.time_from_start += init_duration; 

      // Find where to splice in the new trajectory via binary search
      insertion_bounds = std::equal_range(
          traj_points_.begin(),
          traj_points_.end(),
          first_point,
          point_time_cmp);

      // Clear points after insertion bounds
      traj_points_.erase(insertion_bounds.first, traj_points_.end());

      // Add all trajectory points after the insertion point 
      for(std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it=new_trajectory_.points.begin();
          it != new_trajectory_.points.end();
          it++)
      {
        // Add the begin time to the "time from start"
        it->time_from_start += init_duration;
        // Insert the point
        traj_points_.push_back(*it);
      }

      /*
      // Find where to splice in the new trajectory via binary search
      // Compute start time for this trajectory
      ros::Time new_traj_start_time = new_trajectory_.header.stamp + new_trajectory_.points[0].time_from_start;
      // Initializa binary search indices
      size_t insertion = traj_points_.size()/2,
             low = 0,
             high = traj_points_.size()-1;
      while(insertion > low && insertion < high) {
        if(new_traj_start_time < traj_points_[insertion].time_from_start) {
          high = insertion - 1;
          insertion = (low + insertion)/2;
        } else {
          low = insertion + 1;
          insertion = (insertion + high)/2;
        }
      }
      // Pre-allocate any needed memory
      traj_points_.reserve(
          size_t(insertion_bounds.first - traj_points_.begin()) // # of points to keep
          + new_trajectory_.points.size());                     // # of new points
      */
    }
  } while(trajectories_in_status != RTT::NewData);

  // Check if we should dispatch the point
  ros::Time time = ros::Time::now();
  if(traj_points_.size() > 0 && last_point_.time_from_start < ros::Duration(time.sec, time.nsec)) {
    // Set desired positions
    for(unsigned int i=0; i<n_dof_; i++) {
      positions_des_.q(i) = traj_points_.front().positions[i];
      positions_des_.qdot(i) = traj_points_.front().velocities[i];
    }

    // Send joint positions
    positions_out_port_.write( positions_des_ );
    
    // Store and pop point off the trajectory list
    last_point_ = traj_points_.front();
    traj_points_.pop_front();
  }
}

void JointTrajectory::stopHook()
{
}

void JointTrajectory::cleanupHook()
{
}

