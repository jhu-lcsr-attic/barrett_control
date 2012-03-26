#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <angles/angles.h>

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

  // Store joints from urdf
  joints_.clear();
  for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      it++)
  {
    joints_.push_back(urdf_model_.getJoint(it->getJoint().getName()));
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

// Comparison function for binary search
bool segment_time_cmp(
    JointTrajectory::Segment i,
    JointTrajectory::Segment j)
{
  return (i.start_time < j.start_time);
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
void JointSplineTrajectoryController::sampleSplineWithTimeBounds(
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

void JointTrajectory::load_trajectories_interp(trajectory_msgs::JointTrajectory msg)
{
  // Store the time of the last sampled point that was dispatched
  ros::Time time = last_time_;
  
  // Map from an index in joints_ to an index in the msg
  std::vector<int> lookup(joints_.size(), -1); 
  for (size_t j = 0; j < joints_.size(); ++j) {
    for (size_t k = 0; k < msg->joint_names.size(); ++k) {
      if (msg->joint_names[k] == joints_[j]->name) {
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
  if (msg->points.size() > 0) {
    // Start time is the stamp + time of fist point
    msg_start_time = (msg->header.stamp + msg->points[0].time_from_start).toSec();
  } else {
    // Start time is the stamp (time to truncate the current trajectory)
    msg_start_time = msg->header.stamp.toSec();
  }

  // Declare bounds for binary search
  std::pair<SplineTrajectory::iterator, SplineTrajectory::iterator> insertion_bounds;

  // Construct a dummy segment with the new trajectory's start time
  Segment first_segment;
  first_segment.start_time = msg_start_time;

  // Find where to splice in the new trajectory via binary search
  insertion_bounds = std::equal_range(
      traj_splines_.begin(),
      traj_splines_.end(),
      first_segment,
      segment_time_cmp);

  // Clear points after insertion bounds
  traj_splines_.erase(insertion_bounds.first, traj_splines_.end());

  /////////////////////////////////////////////////////////////////////////////
  // Inerpolate from the last segment
  
  // Declare the previous joint state
  std::vector<double> prev_positions(n_dof_);
  std::vector<double> prev_velocities(n_dof_);
  std::vector<double> prev_accelerations(n_dof_);

  // Get the last segment
  Segment last_segment;
  if(traj_splines_.size() > 0) {
    // Get the segment on the end of the spline trajectory
    last_segment = traj_splines_.back();

    // Find the end conditions of the last segment from the spline trajectory
    ROS_DEBUG("Initial conditions for new set of splines:");
    for (unsigned int i = 0; i < n_dof_; ++i) {
      // Shift the new traj start time into the local time of this segment
      double new_traj_time_in_segment = msg_start_time - last_segment.start_time;

      // Sample a spline at time (msg_start_time-last_segment.start_time) that
      // takes (last_segment.duration) to complete 
      sampleSplineWithTimeBounds(
          last_segment.splines[i].coef,
          last_segment.duration,
          new_traj_start_time_in_segment,
          prev_positions[i],
          prev_velocities[i],
          prev_accelerations[i]);

      ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)",
          prev_positions[i], prev_velocities[i],
          prev_accelerations[i], joints_[i]->joint_->name.c_str());
    }
  } else {
    // Define the initial conditions from the last joint state reading
    for (unsigned int i = 0; i < n_dof_; ++i) {
      prev_positions[i] = positions_.q(i);
      prev_velocities[i] = positions_.qdot(i);
      prev_accelerations[i] = 0.0;
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Construct the segments for the new trajectory

  // Compute durations of each segment (one segment per point)
  std::vector<double> durations(msg->points.size());

  // Duration of segment (i) is the time from point (i-1) to point (i) in seconds
  durations[0] = msg->points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg->points.size(); ++i) {
    durations[i] = (msg->points[i].time_from_start - msg->points[i-1].time_from_start).toSec();
  }

  // Check if each joint should wrap angles between joint limits or not
  std::vector<double> wrap(n_dof_, 0.0);
  assert(!msg->points[0].positions.empty());

  for (size_t j = 0; j < joints_.size(); ++j) {
    if (joints_[j]->type == urdf::Joint::CONTINUOUS) {
      double dist = angles::shortest_angular_distance(
          prev_positions[j],
          msg->points[0].positions[j]);
      wrap[j] = (prev_positions[j] + dist) - msg->points[0].positions[j];
    }
  }
  
  // Declare joint state working variables
  std::vector<double> positions(n_dof_,0.0);
  std::vector<double> velocities(n_dof_,0.0);
  std::vector<double> accelerations(n_dof_,0.0);

  // Construct spline segment for each point
  for (size_t i = 0; i < msg->points.size(); ++i) {
    // Construct new segment
    Segment seg;

    // Compute absolute start time
    seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
    seg.duration = durations[i];
    seg.splines.resize(joints_.size());

    // Checks that the incoming point has the right number of elements.
    if (msg->points[i].accelerations.size() != 0 && msg->points[i].accelerations.size() != joints_.size()) {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)msg->points[i].accelerations.size());
      return;
    }
    if (msg->points[i].velocities.size() != 0 && msg->points[i].velocities.size() != joints_.size()) {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)msg->points[i].velocities.size());
      return;
    }
    if (msg->points[i].positions.size() != joints_.size()) {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)msg->points[i].positions.size());
      return;
    }

    // Re-orders the joints in the command to match the interal joint order.
    accelerations.resize(msg->points[i].accelerations.size());
    velocities.resize(msg->points[i].velocities.size());
    positions.resize(msg->points[i].positions.size());
    for (size_t j = 0; j < joints_.size(); ++j) {
      if (!accelerations.empty()) {
        accelerations[j] = msg->points[i].accelerations[lookup[j]];
      }
      if (!velocities.empty()) {
        velocities[j] = msg->points[i].velocities[lookup[j]];
      }
      if (!positions.empty()) {
        positions[j] = msg->points[i].positions[lookup[j]] + wrap[j];
      }
    }

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
    traj_splines_.push_back(seg);

    // Store the initial conditions for the next segment
    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }
  
  ROS_DEBUG("Spliced in new trajectory.");
}
      
void JointTrajectory::load_trajectories()
{
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
  
}

void JointTrajectory::updateHook()
{
  // Read in the current joint positions & velocities
  positions_in_port_.readNewest( positions_ );

  // Update time metrics
  ros::Time time = util::rtt_ros_now();
  last_time_ = time;

  // Iterate through segments while the next segment starts before the current time
  SplineTrajectory::iterator seg_it = traj_splines_.begin(); 

  while((seg_it+1) != traj_splines_.end() && (seg_it+1)->start_time < time.toSec()) {
    seg_it++;
  }
    
  // Check if we have reached the end of the trajectory
  if((seg_it+1) == traj_splines_.end()) {
    ROS_DEBUG("End of trajectory reached.");
    return;
  }
    
  // Compute the time in this segment
  double seg_time = time.toSec() - seg_it->start_time; 
  // Sample from the current segment in the trajectory
  for (size_t i = 0; i < q.size(); ++i) {
    sampleSplineWithTimeBounds(
        seg_it->splines[i].coef,
        seg_it->duration,
        seg_time,
        positions_des_.q(i),
        positions_des_.qdot(i),
        pva_des_.qdotdot(i));
  }

  // Dispatch the sampled point
  positions_out_port_.write( positions_des_ );

  // Store the current segment
  last_segment_ = *seg_it;
  
  // Clear finished segments
  traj_splines_.erase(traj_splines_.begin(), (seg_it-1));
}

void JointTrajectory::stopHook()
{
}

void JointTrajectory::cleanupHook()
{
}

