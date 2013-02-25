#include <ros/ros.h>
#include <native/task.h>
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <barrett_hw/wam.h>
#include <controller_manager/controller_manager.h>
#include <signal.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Duration.h>

bool g_quit = false;

void quitRequested(int sig) {
  g_quit = true;
}

int main( int argc, char** argv ){

  // Set up real-time task
  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupWAM", 99, 0 );

  // Initialize ROS
  ros::init(argc, argv, "wam_server", ros::init_options::NoSigintHandler);

  // Add custom signal handlers
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Construct the wam structure
  ros::NodeHandle wam_nh("wam");
  barrett_hw::WAM wam_hw( wam_nh );

  // Timer variables
  struct timespec ts = {0,0};

  if(clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    ROS_FATAL("Failed to poll realtime clock!");
  }

  ros::Time 
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(0.0);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  realtime_tools::RealtimePublisher<std_msgs::Duration> publisher(wam_nh, "loop_rate", 2);

  if(!wam_hw.configure()) {
    ROS_FATAL("Could not configure WAM!");
    return -1;
  }

  if(!wam_hw.start()) {
    ROS_FATAL("Could not start WAM!");
    return -1;
  }

  // Construct the controller manager
  ros::NodeHandle nh;
  controller_manager::ControllerManager manager(&wam_hw, nh);

  uint32_t count = 0;

  // Run as fast as possible
  while( !g_quit ) {
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

    if(count++ > 1000) {
      if(publisher.trylock()) {
        count = 0;
        publisher.msg_.data = period;
        publisher.unlockAndPublish();
      }
    }
  }

  publisher.stop();

  std::cerr<<"Stpping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Stopping WAM..."<<std::endl;
  wam_hw.stop();

  std::cerr<<"Cleaning up WAM..."<<std::endl;
  wam_hw.cleanup();

  std::cerr<<"Poka!"<<std::endl;

  return 0;
}

