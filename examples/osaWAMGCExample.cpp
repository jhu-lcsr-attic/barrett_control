#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawBarrett/osaWAM.h>
#include <sawCANBus/osaRTSocketCAN.h>
#include <sawControllers/osaGravityCompensation.h>

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

  osaWAM WAM( &can );

  if( WAM.Initialize() != osaWAM::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize WAM" << std::endl;
    return -1;
  }

  vctDynamicVector<double> qinit( 7, 0.0 );
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;
  
  if( WAM.SetPositions( qinit ) != osaWAM::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to set position: " << qinit << std::endl;
    return -1;
  }

  std::string path(  CISST_SOURCE_ROOT"/cisst/etc/cisstRobot/" );

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );
  
  osaGravityCompensation GC( path+"WAM/wam7.rob", Rtw0 );

  std::cout << "Activate the WAM" << std::endl;
  bool activated = false;

  double t1 = osaGetTime();
  size_t cnt=0;

  while( 1 ){

    // Get the positions
    vctDynamicVector<double> q;
    if( WAM.GetPositions( q ) != osaWAM::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to get positions" << std::endl;
      return -1;
    }

    // Check if the pucks are activated
    if( !activated ) {
      osaWAM::Mode mode;
      if( WAM.GetMode( mode ) != osaWAM::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to get mode" << std::endl;
	return -1;
      }
      if( mode == osaWAM::MODE_ACTIVATED )
	{ activated = true; }
    }

    // if pucks are activated, run the controller
    vctDynamicVector<double> tau( q.size(), 0.0 );
    if( activated ){
      if( GC.Evaluate( q, tau ) != osaGravityCompensation::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to evaluate controller" << std::endl;
	return -1;
      }
    }

    // apply torques
    if( WAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
      return -1;
    }

    std::cout << "q:   " << q << std::endl;
    std::cout << "tau: " << tau << std::endl;

    cnt++;
    if( cnt == 1000 ){
      double t2 = osaGetTime();
      std::cout << 1000.0 / (t2 - t1) << std::endl;
      t1 = t2;
      cnt = 0;
    }

  }

  return 0;
}
