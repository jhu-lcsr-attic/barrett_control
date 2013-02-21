#include <barrett_direct/WAM.h>
#include <leoCAN/RTSocketCAN.h>
#include <native/task.h>
#include <sys/mman.h>
#include <cmath>
#include <time.h>

using namespace leoCAN;
using namespace barrett_direct;

int main( int argc, char** argv ){

  // Set up real-time task
  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupTest", 99, 0 );

  // Check arguments
  if( argc != 2 ){
    std::cout << "Usage: " << argv[0] << " rtcan[0-1]" << std::endl;
    return -1;
  }

  // Create CANBus device
  RTSocketCAN can( argv[1], CANBus::RATE_1000 );
  // Make sure the device is open
  if( can.Open() != leoCAN::CANBus::ESUCCESS ){
    std::cerr << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  // Construct the wam structure
  WAM wam_robot( &can );
  // Make sure it initializaes successfully
  if( wam_robot.Initialize() != WAM::ESUCCESS ){
    std::cerr << "Failed to initialize WAM" << std::endl;
    return -1;
  }

  // Create a joint position vector for the calibration position
  Eigen::VectorXd q_init(7);
  q_init.setConstant( 0.0 );
  q_init[1] = -M_PI_2;
  q_init[3] =  M_PI;
  
  // Set the calibration position
  if( wam_robot.SetPositions( q_init ) != WAM::ESUCCESS ){
    std::cerr << "Failed to set position: " << q_init << std::endl;
    return -1;
  }

  // Timer variables
  size_t cnt = 0;

  struct timespec ts1 = {0,0} ,ts2 = {0,0};
  time_t sec_diff = 0;
  long nsec_diff = 0;
  int ret = clock_gettime(CLOCK_REALTIME, &ts1);
  if(ret) {
    std::cerr << "Failed to poll realtime clock!"<<std::endl;
  }

  // Joint position vector and torque vector
  Eigen::VectorXd q( q_init.size());
  Eigen::VectorXd tau( q.size());

  // Run as fast as possible
  while( true ) {

    // Get the posisionts
    if( wam_robot.GetPositions( q ) != WAM::ESUCCESS ){
      std::cerr << "Failed to get positions" << std::endl;
      return -1;
    }

    // Set the torques to zero
    tau.setConstant( 0.0 );
    if( wam_robot.SetTorques( tau ) != WAM::ESUCCESS ){
      std::cerr << "Failed to set torques" << std::endl;
      return -1;
    }

    // Display the arm joint positions
    std::cout << "q: " << q << std::endl;

    // Increment the counter and display the loop rate
    cnt++;
    if( cnt == 1000 ){
      clock_gettime(CLOCK_REALTIME, &ts2);
      if (!ret) {
        sec_diff = ts2.tv_sec - ts1.tv_sec ;
        nsec_diff = ts2.tv_nsec - ts1.tv_nsec;
      }
      std::cout << 1000.0 / ((double)sec_diff + 1.0E-9*(double)nsec_diff) << " Hz" << std::endl;
      ts1 = ts2;
      cnt = 0;
    }

  }

  return 0;
}
