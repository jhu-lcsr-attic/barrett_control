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
 *
 * This code has been adapted from the PR2 JointSplineTrajectoryController,
 * developed at Willow Garage, Inc. for the PR2 Robot. It has been heavily
 * modified, but retains some of the mathematical computations for spline
 * interpolation and general algorithmic structure. The following is included as
 * part of that code's BSD license:
 *
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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


#include <iostream>
#include <sstream>
#include <map>
#include <algorithm>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <angles/angles.h>

#include <bard_components/util.h>
#include <bard_components/controllers/joint_trajectory.h>

#define _TICTOC (0)

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
  this->addProperty("velocity_limits",velocity_limits_).doc("The velocity limits.");

  // Configure data ports
  this->ports()->addEventPort("positions_in", positions_in_port_, boost::bind(&JointTrajectory::feedback_cb, this)).doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addEventPort("trajectories_in", trajectories_in_port_, boost::bind(&JointTrajectory::command_cb, this)).doc("Input port: nx1 vector of desired joint positions. (n joints)");
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

  // Store joints from urdf
  joints_.clear();
  velocity_limits_.clear();
  for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      it++)
  {
    joints_.push_back(urdf_model_.getJoint(it->getJoint().getName()));
    velocity_limits_.push_back(joints_.back()->limits->velocity);
  }

  // Resize working vectors
  positions_.resize(n_dof_);
  positions_des_.resize(n_dof_);
  pva_des_.resize(n_dof_);

  KDL::SetToZero(positions_.q);
  KDL::SetToZero(positions_.qdot);

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_des_);

  return true;
}

bool JointTrajectory::startHook()
{
  // Reset the trajectory
  spline_traj_.clear();
  active_segment_it_ = spline_traj_.begin();
  traj_count_ = 0;
  
  // Clear input ports
  positions_in_port_.clear();
  trajectories_in_port_.clear();

  return true;
}

// Comparison function for binary search
bool point_time_cmp(
    trajectory_msgs::JointTrajectoryPoint i,
    trajectory_msgs::JointTrajectoryPoint j)
{
  return (i.time_from_start < j.time_from_start);
}

// Comparison function for binary search
bool segment_time_cmp(
    JointTrajectory::Segment i,
    JointTrajectory::Segment j)
{
  return (i.end_time < j.end_time);
}

// Generate an array of powers of a given value
static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++) {
    powers[i] = powers[i-1]*x;
  }
}

// Compute the coefficients for a quintic spline
static void getQuinticSplineCoefficients(
    double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc,
    double time,
    std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0) {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5*end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  } else {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
}

// Sample a quintic spline segment at a particular time
static void sampleQuinticSpline(
    const std::vector<double>& coefficients,
    double time,
    double& position,
    double& velocity,
    double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
      t[1]*coefficients[1] +
      t[2]*coefficients[2] +
      t[3]*coefficients[3] +
      t[4]*coefficients[4] +
      t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
      2.0*t[1]*coefficients[2] +
      3.0*t[2]*coefficients[3] +
      4.0*t[3]*coefficients[4] +
      5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
      6.0*t[1]*coefficients[3] +
      12.0*t[2]*coefficients[4] +
      20.0*t[3]*coefficients[5];
}

// Compute the coefficients for a cubic spline
static void getCubicSplineCoefficients(
    double start_pos, double start_vel,
    double end_pos, double end_vel,
    double time, std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0) {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  } else {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
}

// Sample a quintic spline with a certain duration at a given time
void sampleSplineWithTimeBounds(
    const std::vector<double>& coefficients,
    double duration,
    double time,
    double& position,
    double& velocity,
    double& acceleration)
{
  // Discarded value
  double _;

  if (time < 0) {
    // Sample is before the beginning of the spline
    sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  } else if (time > duration) {
    // Sample is after the end of the spline
    sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  } else {
    // Sample is during the execution of the spline
    sampleQuinticSpline(
        coefficients, time,
        position, velocity, acceleration);
  }
}

template <typename T>
std::string iter_name(T &l, typename T::iterator &it) {
  if(it == l.begin()) {
    return std::string("BEGIN");
  } else if(it == l.end()) {
    return std::string("END");
  } else {
    return std::string("OTHER");
  }
}

void JointTrajectory::command_cb()
{
  ROS_DEBUG("Received new trajecotory.");
  //RTT::os::MutexLock lock(traj_cmd_mutex_);

  // Read in the new message
  trajectory_msgs::JointTrajectory msg;
  trajectories_in_port_.readNewest(msg);

  // Store the time of the last sampled point that was dispatched
  ros::Time time = last_time_;
  
  // Construct a map from an index in joints_ to an index in the msg
  ROS_DEBUG("Constructing index map.");
  std::vector<int> lookup(joints_.size(), -1); 
  for (size_t j = 0; j < joints_.size(); ++j) {
    for (size_t k = 0; k < msg.joint_names.size(); ++k) {
      if (msg.joint_names[k] == joints_[j]->name) {
        lookup[j] = k;
        break;
      }
    }
    if (lookup[j] == -1) {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joints_[j]->name.c_str());
      return;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Find the last segment that happens before the new trajecotry is supposed
  // to start.

  // Compute the trajectory start time
  double msg_start_time;
  SplineTrajectory::iterator insertion_it = spline_traj_.end();

  if (msg.points.size() == 0) {
    ROS_WARN("Empty trajectory!");
    return;
  }

  ROS_DEBUG_STREAM("Trajectory start time is: "<<msg.header.stamp);

  if(msg.header.stamp.isZero()) {
    ROS_DEBUG("Starting trajectory immediately,");

    // Start immediately
    msg_start_time = util::ros_rtt_now().toSec()+ros::Duration(0.002);

    // Insert after the first segment
    insertion_it = spline_traj_.begin();
  } else {
    // Compute the time at which this trajectory should start
    msg_start_time = msg.header.stamp.toSec();

    ROS_DEBUG_STREAM("Starting trajectory in "<<(msg_start_time-util::ros_rtt_now().toSec())<<" seconds.");

    // Construct a dummy segment with the new trajectory's start time
    Segment first_new_segment;

    // Set the duration of the first segment
    first_new_segment.duration = msg.points[0].time_from_start.toSec();

    // Compute the end time of the new segment
    first_new_segment.end_time = msg_start_time + first_new_segment.duration;
    
    // Declare bounds for binary search
    std::pair<SplineTrajectory::iterator, SplineTrajectory::iterator> insertion_bounds;

    // Find where to splice in the new trajectory via binary search
    // Performs O(log n) comparisons, but std::distance and std::advance are not
    // constant time since we're using an std::list
    SplineTrajectory::iterator
      lower = spline_traj_.begin(),
      upper = spline_traj_.end(),
      pivot;

    ROS_DEBUG_STREAM("Binary search for insertion point...");
    int middle = 0;
    do {
      // Compute the distance to the middle of the two bounds
      middle = std::distance(lower,upper)/2;
      ROS_DEBUG_STREAM("  middle: "<<middle);

      // Set the pivot to the lower bound and then advance it to the middle
      pivot = lower;
      std::advance(pivot,middle);

      // Check if the new segment needs to be inserted before or after the pivot
      if(first_new_segment.end_time < pivot->end_time) {
        // Set the upper bound to one less than the pivot
        ROS_DEBUG_STREAM("  below pivot: "<<first_new_segment.end_time<<" < "<<pivot->end_time);
        upper = --pivot;
      } else {
        // Set the lower bound to one more than the pivot
        ROS_DEBUG_STREAM("  above pivot: "<<first_new_segment.end_time<<" > "<<pivot->end_time);
        lower = ++pivot;
      }
    } while(middle != 0); // Until lower and upper are adjacent

    /*                 
    ROS_DEBUG_STREAM("Insertion lower of spline_traj_ "<<iter_name<std::list<Segment> >(spline_traj_, lower));
    ROS_DEBUG_STREAM("Insertion pivot of spline_traj_ "<<iter_name<std::list<Segment> >(spline_traj_, pivot));
    ROS_DEBUG_STREAM("Insertion upper of spline_traj_ "<<iter_name<std::list<Segment> >(spline_traj_, upper));
    std::ostringstream oss;
    for(std::list<Segment>::iterator it = spline_traj_.begin();
        it != spline_traj_.end();
        ++it)
    {
      if(it == lower) {
        oss<<"("<<it->traj_index<<"."<<it->index<<" ";
      } else if(it == upper) {
        oss<<it->traj_index<<"."<<it->index<<") ";
      } else {
        oss<<it->traj_index<<"."<<it->index<<" ";
      }
    }

    ROS_DEBUG_STREAM("[ "<<oss.str()<<"]");
    */
    if(upper == spline_traj_.end() ) {
      // Set insertion iterator to the last segment of the current trajectory
      // (this should be equivalent to (--spline_traj_.end()))
      insertion_it = --upper;
    } else {
      // Set insertion iterator to the segment that ends after the insertion time
      insertion_it = upper;
    }
  }
  // POST: insertion_it points to a valid Segment structure if there is an active trajectory

  /////////////////////////////////////////////////////////////////////////////
  // Inerpolate from the segment preceding the new trajectory
  
  // Declare the previous joint state
  std::vector<double> prev_positions(n_dof_);
  std::vector<double> prev_velocities(n_dof_);
  std::vector<double> prev_accelerations(n_dof_);

  // Check if there is currently a trajectory being executed
  // TODO: make sure we're actually executing it!
  if( active_segment_it_ != spline_traj_.end() ) {
    ROS_DEBUG("New trajectory will interrupt current trajectory.");

    // Truncate the duration & end time of the interrupted segment
    insertion_it->duration = insertion_it->duration - (insertion_it->end_time - msg_start_time);
    insertion_it->end_time = msg_start_time;

    // Get the initial conditions from the start time of the new traj
    ROS_DEBUG("Initial conditions for new set of splines:");
    for (unsigned int i = 0; i < n_dof_; ++i) {
      // Shift the new traj start time into the local time of this segment
      double new_traj_time_in_segment =
        msg_start_time - (insertion_it->end_time - insertion_it->duration);

      // Sample a spline at time (msg_start_time-last_segment.end_time) that
      // takes (last_segment.duration) to complete 
      sampleSplineWithTimeBounds(
          insertion_it->splines[i].coef,
          insertion_it->duration,
          new_traj_time_in_segment,
          prev_positions[i],
          prev_velocities[i],
          prev_accelerations[i]);

      ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)",
          prev_positions[i], prev_velocities[i],
          prev_accelerations[i], joints_[i]->name.c_str());
    }
  } else {
    ROS_DEBUG("No active trajectory. Defining initial conditions from last position reading:");
    // Define the initial conditions from the last joint state reading
    ROS_DEBUG("Initial conditions for new set of splines:");
    for (unsigned int i = 0; i < n_dof_; ++i) {
      prev_positions[i] = positions_.q(i);
      prev_velocities[i] = positions_.qdot(i);
      prev_accelerations[i] = 0.0;

      ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)",
          prev_positions[i], prev_velocities[i],
          prev_accelerations[i], joints_[i]->name.c_str());
    }
  }
  // POST: prev_* contain the initial condtions for the first segmet of the new traj
  //

  /////////////////////////////////////////////////////////////////////////////
  // Construct the segments for the new trajectory

  // New trajectory segments
  SplineTrajectory new_spline_traj;

  // Compute durations of each segment (one segment per point)
  std::vector<double> durations(msg.points.size());

  // Duration of segment (i) is the time from point (i-1) to point (i) in seconds
  durations[0] = msg.points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg.points.size(); i++) {
    durations[i] =
      (msg.points[i].time_from_start - msg.points[i-1].time_from_start).toSec();
  }

  // Check if each joint should wrap angles between joint limits or not
  std::vector<double> wrap(n_dof_, 0.0);
  assert(!msg.points[0].positions.empty());
  
  for (size_t j = 0; j < n_dof_; j++) {
    if (joints_[j]->type == urdf::Joint::CONTINUOUS) {
      double dist = angles::shortest_angular_distance(
          prev_positions[j],
          msg.points[0].positions[j]);
      wrap[j] = (prev_positions[j] + dist) - msg.points[0].positions[j];
    }
  }
  
  // Declare joint state working variables
  std::vector<double> positions(n_dof_,0.0);
  std::vector<double> velocities(n_dof_,0.0);
  std::vector<double> accelerations(n_dof_,0.0);

  // Initialize safety time offset
  // This is incremented to slow down a trajectory to live within velocity limits
  double safety_time_offset = 0.0;

  // Construct spline segment for each point
  for (size_t i = 0; i < msg.points.size(); i++) {

    // Checks that the incoming point has the right number of elements.
    if (msg.points[i].accelerations.size() != 0 && msg.points[i].accelerations.size() != joints_.size()) {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)msg.points[i].accelerations.size());
      return;
    }
    if (msg.points[i].velocities.size() != 0 && msg.points[i].velocities.size() != joints_.size()) {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)msg.points[i].velocities.size());
      return;
    }
    if (msg.points[i].positions.size() != joints_.size()) {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)msg.points[i].positions.size());
      return;
    }

    // Re-order the joints in the command to match the interal joint order.
    accelerations.resize(msg.points[i].accelerations.size());
    velocities.resize(msg.points[i].velocities.size());
    positions.resize(msg.points[i].positions.size());
    for (size_t j = 0; j < joints_.size(); ++j) {
      if (!accelerations.empty()) { accelerations[j] = msg.points[i].accelerations[lookup[j]]; }
      if (!velocities.empty()) {    velocities[j] = msg.points[i].velocities[lookup[j]]; }
      if (!positions.empty()) {     positions[j] = msg.points[i].positions[lookup[j]] + wrap[j]; }
    }
    
    // Construct new segment
    Segment seg;

    // Compute absolute start time
    seg.traj_index = traj_count_;
    seg.index = i;
    seg.duration = durations[i];
    seg.end_time = msg_start_time + msg.points[i].time_from_start.toSec();
    seg.splines.resize(joints_.size());

    // Check the maximum average velocity of this segment
    double max_vel_ratio = 0.0;
    for(size_t j = 0; j < n_dof_; j++) {
      max_vel_ratio = std::max(max_vel_ratio, (fabs(prev_positions[j] - positions[j])/seg.duration)/velocity_limits_[j]);
    }
    ROS_DEBUG_STREAM("Maximum velocity ratio is: "<<max_vel_ratio);

    // Compute new (safe) duration
    double new_duration = std::max(seg.duration, max_vel_ratio*seg.duration);
    ROS_DEBUG_STREAM("Duration update: "<<seg.duration<<"->"<<new_duration);

    // Increment time offset
    safety_time_offset += new_duration-seg.duration;

    // Update time properties to account for safety offset
    durations[i] = new_duration;
    seg.duration = new_duration;
    seg.end_time += safety_time_offset;
    
    // Compute splines from the boundary conditions
    for (size_t j = 0; j < n_dof_; ++j) {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0) {
        // Compute splines with pos,vel,acc
        getQuinticSplineCoefficients(
          prev_positions[j], prev_velocities[j], prev_accelerations[j],
          positions[j], velocities[j], accelerations[j],
          durations[i],
          seg.splines[j].coef);

      } else if (prev_velocities.size() > 0 && velocities.size() > 0) {
        // Compute splines with pos,vel
        getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          positions[j], velocities[j],
          durations[i],
          seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);

      } else {
        // Compute splines with position only
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0) {
          seg.splines[j].coef[1] = 0.0;
        } else {
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        }
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }

    // Push the splines onto the end of the new trajectory.
    new_spline_traj.push_back(seg);

    // Store the initial conditions for the next segment
    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  // Increment insertion it to the following segment
  ++insertion_it;

  // Delete the no longer valid trajectory
  ROS_DEBUG_STREAM("Erasing invalid segments from trajectory ("<<spline_traj_.size()<<")");
  if(insertion_it == spline_traj_.end()) {
    ROS_DEBUG_STREAM("Erasing no segments.");
  } else {
    ROS_DEBUG_STREAM("Erasing segments starting with index "<<insertion_it->index);
  }
  spline_traj_.erase(insertion_it, spline_traj_.end());

  // Splice in the new trajectory
  ROS_DEBUG_STREAM("Splicing "<<new_spline_traj.size()<<" new segment(s) into trajectory.");
  spline_traj_.splice(spline_traj_.end(), new_spline_traj);

  ROS_DEBUG_STREAM("Trajectory now has "<<spline_traj_.size()<<" segments.");

#if _TICTOC
#else
  // Remove old segments
  SplineTrajectory::iterator last_old_segment_it = active_segment_it_;
  --last_old_segment_it;
  spline_traj_.erase(spline_traj_.begin(), last_old_segment_it);

  ROS_DEBUG_STREAM("Trajectory now has "<<spline_traj_.size()<<" segments.");
#endif

  // Increment trajectory counter (just used for identifying different traj segments)
  traj_count_++;
}

void JointTrajectory::feedback_cb()
{
  // Read in the current joint positions & velocities
  positions_in_port_.readNewest( positions_ );

  // Update time metrics
  ros::Time now = util::ros_rtt_now();

  if(spline_traj_.size()>0) {
    if(last_time_.toSec() < spline_traj_.back().end_time
        && now.toSec() > spline_traj_.back().end_time)
    {
      //ROS_DEBUG_STREAM("End of trajectory reached. Size: "<<spline_traj_.size());
    }
  }

  last_time_ = now;

  //RTT::os::MutexLock lock(traj_cmd_mutex_);
#if _TICTOC
  active_segment_it_ = spline_traj_.begin();
#else
#endif

  // Iterate through segments to find the active one
  while(active_segment_it_ != spline_traj_.end()) {
    // Check if this segment begins later than now
    if(active_segment_it_->end_time > now.toSec()) {
      //ROS_DEBUG_STREAM("Active segment found! End time: "<<active_segment_it_->end_time);
      break;
    } else {
      //ROS_DEBUG_STREAM("Active end time: "<<active_segment_it_->end_time<<" but it is currently "<<now);
    }

    // Clear the old segments
    spline_traj_.pop_front();
#if _TICTOC
    // Reset the iterator
    active_segment_it_ = spline_traj_.begin();
#else
    // TODO: try this instead
    // This could keep the trajectory structure from changing when we dont have
    // a command. Then, we could just remove the old segments when the command
    // comes in.
    ++active_segment_it_;
#endif
  } 

  // Check if we have reached the end of the trajectory
  if(active_segment_it_ == spline_traj_.end()) {
    return;
  }

  // Compute the time from the beginning of this segment
  double seg_time = now.toSec() - (active_segment_it_->end_time-active_segment_it_->duration); 

  //ROS_DEBUG_STREAM("Sampling segment at t="<<seg_time<<" s:");
  //ROS_DEBUG_STREAM("  end_time: "<<active_segment_it_->end_time);
  //ROS_DEBUG_STREAM("    duration: "<<active_segment_it_->duration);
  //ROS_DEBUG_STREAM("    #splines: "<<active_segment_it_->splines.size());

  // Sample from the current segment in the trajectory
  for (size_t i = 0; i < n_dof_; i++) {
    sampleSplineWithTimeBounds(
        active_segment_it_->splines[i].coef,
        active_segment_it_->duration,
        seg_time,
        positions_des_.q(i),
        positions_des_.qdot(i),
        pva_des_.qdotdot(i));
  }
  
  // Dispatch the sampled point
  positions_out_port_.write( positions_des_ );
  
  //ROS_DEBUG_STREAM("Done.");
}
      

void JointTrajectory::updateHook()
{
}

void JointTrajectory::stopHook()
{
}

void JointTrajectory::cleanupHook()
{
}
