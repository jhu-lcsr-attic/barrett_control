#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawBarrett/osaBH8_280.h>
#include <sawCANBus/osaRTSocketCAN.h>
#include <native/task.h>
#include <sys/mman.h>

int main( int argc, char** argv ){

  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupTest", 99, 0 );

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 2 ){
    std::cout << "Usage: " << argv[0] << " rtcan[0-1]" << std::endl;
    return -1;
  }

  osaRTSocketCAN can( argv[1], osaCANBus::RATE_1000 );

  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  osaBH8_280 BH( &can );

  if( BH.Initialize() != osaBH8_280::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize WAM" << std::endl;
    return -1;
  }
  std::cout << "\n\n\n\n";
  double t1 = osaGetTime();
  size_t cnt=0;

  while( 1 ){

    Eigen::VectorXd q( 4, 0.1 );

    BH.GetPositions( q );
    //BH.SetPositions( q );

    }
    cmnGetChar();
  return 0;
}
