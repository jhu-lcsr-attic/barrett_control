#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawBarrett/osaWAM.h>
#include <sawCANBus/osaRTSocketCAN.h>
#include <sawControllers/osaPDGC.h>

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

  Eigen::VectorXd qinit( 7, 0.0 );
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;
  
  if( WAM.SetPositions( qinit ) != osaWAM::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to set position: " << qinit << std::endl;
    return -1;
  }

  std::string path(  CISST_SOURCE_ROOT"/cisst/etc/cisstRobot/" );

  // Rotate the base
  Eigen::Matrix3d Rw0(  0.0,  0.0, -1.0,
                        0.0,  1.0,  0.0,
                        1.0,  0.0,  0.0 );
  Eigen::Affine3d Rtw0( Eigen::Rotation3f(Rw0) );
  
  // Gain matrices
  Eigen::MatrixXd Kp(7, 7, 0.0), Kd(7, 7, 0.0);
  Kp[0][0] = 250;     Kd[0][0] = 3.0;
  Kp[1][1] = 250;     Kd[1][1] = 3.0;
  Kp[2][2] = 250;     Kd[2][2] = 3.0;
  Kp[3][3] = 200;     Kd[3][3] = 3;
  Kp[4][4] = 50;      Kd[4][4] = 0.8;
  Kp[5][5] = 50;      Kd[5][5] = 0.8;
  Kp[6][6] = 10;      Kd[6][6] = .1;

  osaPDGC PDGC( path+"WAM/wam7.rob", Rtw0, Kp, Kd, qinit );

  std::cout << "Activate the WAM" << std::endl;
  bool activated = false;

  double t1 = osaGetTime();

  while( 1 ){

    // Get the positions
    Eigen::VectorXd q;
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
    Eigen::VectorXd tau( q.size(), 0.0 );
    double t2 = osaGetTime();
    if( activated ){
      if( PDGC.Evaluate( qinit, q, tau, t2-t1 ) != osaPDGC::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to evaluate controller" << std::endl;
	return -1;
      }
    }
    t1 = t2;

    // apply torques
    if( WAM.SetTorques( tau ) != osaWAM::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to set torques" << std::endl;
      return -1;
    }

    std::cout << "q:   " << q << std::endl;
    std::cout << "tau: " << tau << std::endl;

  }

  return 0;
}
