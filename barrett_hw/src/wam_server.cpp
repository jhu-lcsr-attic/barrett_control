#include <ros/ros.h>
#include <native/task.h>
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <barrett_hw/wam.h>
#include <controller_manager/controller_manager.h>

int main( int argc, char** argv ){

  // Set up real-time task
  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupWAM", 99, 0 );

  // Initialize ROS
  ros::init(argc, argv, "wam_server");

  // Construct the wam structure
  ros::NodeHandle wam_nh("wam");
  barrett_hw::WAM wam_hw( wam_nh );

  // Construct the controller manager
  ros::NodeHandle nh;
  controller_manager::ControllerManager manager(&wam_hw, nh);

  // Timer variables
  struct timespec ts = {0,0};

  if(clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    ROS_FATAL("Failed to poll realtime clock!");
  }

  ros::Time 
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(0.0);

  // Run as fast as possible
  while( ros::ok() ) {
    // Get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } else {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }

    // Read the state from the WAM
    wam_hw.read(now, period);

    // Update the controllers
    manager.update(now, period);

    // Write the command to the WAM
    wam_hw.write(now, period);
  }

  return 0;
}

