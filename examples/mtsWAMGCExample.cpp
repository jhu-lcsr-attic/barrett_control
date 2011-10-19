#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>

#include <sawKeyboard/mtsKeyboard.h>
#include <sawCANBus/osaRTSocketCAN.h>
#include <sawBarrett/mtsWAM.h>
#include <sawControllers/mtsGravityCompensation.h>

#include <native/task.h>
#include <sys/mman.h>

int main( int argc, char** argv ){

  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "GroupTest", 99, 0 );

  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  if( argc != 2 ){
    std::cout << "Usage: " << argv[0] << " rtcan[0-1]" << std::endl;
    return -1;
  }

  mtsKeyboard kb;
  kb.SetQuitKey( 'q' );
  kb.AddKeyWriteFunction( 'G', "GCEnable", "Enable", true );
  taskManager->AddComponent( &kb );


  osaRTSocketCAN can( argv[1], osaCANBus::RATE_1000 );

  if( can.Open() != osaCANBus::ESUCCESS ){
    CMN_LOG_RUN_ERROR << argv[0] << "Failed to open " << argv[1] << std::endl;
    return -1;
  }

  mtsWAM WAM( "WAM", &can, osaWAM::WAM_7DOF, OSA_CPU4, 80 );
  WAM.Configure();
  WAM.SetPositions( vctDynamicVector<double>(7, 
					     0.0, -cmnPI_2, 0.0, cmnPI, 
					     0.0, 0.0, 0.0 ) );
  taskManager->AddComponent( &WAM );

  std::string path(  CISST_SOURCE_ROOT"/cisst/etc/cisstRobot/" );

  // Rotate the base
  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );
   
  mtsGravityCompensation GC( "GC", 
			     0.002,
			     path+"WAM/wam7.rob", 
			     Rtw0,
			     OSA_CPU3 );
  taskManager->AddComponent( &GC );

  if( !taskManager->Connect( kb.GetName(), "GCEnable",GC.GetName(),"Control") ){
    std::cout << "Failed to connect: " 
	      << kb.GetName() << "::GCEnable to "
	      << kb.GetName()  << "::Control" << std::endl;
    return -1;
  }

  if( !taskManager->Connect( WAM.GetName(), "Input", GC.GetName(), "Output" ) ){
    std::cout << "Failed to connect: " 
	      << WAM.GetName() << "::Input to "
	      << GC.GetName()  << "::Output" << std::endl;
    return -1;
  }

  if( !taskManager->Connect( WAM.GetName(), "Output", GC.GetName(), "Input" ) ){
    std::cout << "Failed to connect: " 
	      << WAM.GetName() << "::Output to "
	      << GC.GetName()  << "::Input" << std::endl;
    return -1;
  }

  taskManager->CreateAll();
  taskManager->StartAll();

  pause();

  taskManager->KillAll();
  taskManager->Cleanup();

  return 0;
}
