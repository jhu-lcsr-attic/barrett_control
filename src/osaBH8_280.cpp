/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawBarrett/osaBH8_280.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstCommon/cmnLogger.h>

// main constructor
osaBH8_280::osaBH8_280(	osaCANBus* canbus ) :
  // create the groups
  //broadcast(      osaGroup::BROADCAST,         canbus, false ),
  hand(           osaGroup::HAND,              canbus, false ),
  handposition(   osaGroup::HAND_POSITION,      canbus ),

  qinit( qinit ) {

  //
  //canbus->AddFilter( osaCANBus::Filter( 0x051F, 0x0403 ) );

  // create the pucks
  pucks.push_back( osaPuck( osaPuck::PUCK_IDF1, canbus ) );
  pucks.push_back( osaPuck( osaPuck::PUCK_IDF2, canbus ) );
  pucks.push_back( osaPuck( osaPuck::PUCK_IDF3, canbus ) );
  pucks.push_back( osaPuck( osaPuck::PUCK_IDF4, canbus ) );


  if( canbus == NULL )
    { CMN_LOG_RUN_ERROR << "CAN device missing" << std::endl; }

}

osaBH8_280::~osaBH8_280(){}

osaBH8_280::Errno osaBH8_280::Initialize(){

  // initialize each puck
  for( size_t i=0; i<pucks.size(); i++ ){
    if( pucks[i].InitializeMotor() != osaPuck::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to initialize puck " << pucks[i].GetID()
			<< std::endl;
      return osaBH8_280::EFAILURE;
    }
  }

  /*
  // initialize the broadcast group
  if( broadcast.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize broadcast group"
		      << std::endl;
    return osaBH8_280::EFAILURE;
  }
  */

  // initialize the upper arm groups
  if( hand.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize hand group"
		      << std::endl;
    return osaBH8_280::EFAILURE;
  }
  
  if( handposition.Initialize() != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to initialize hand positions group"
		      << std::endl;
    return osaBH8_280::EFAILURE;
  }

  
  // initialize the 4x4 transform matrices
  mpos2jpos.SetSize( 4, 4, VCT_ROW_MAJOR );
  jpos2mpos.SetSize( 4, 4, VCT_ROW_MAJOR );
  jtrq2mtrq.SetSize( 4, 4, VCT_ROW_MAJOR );
  
  mpos2jpos.SetAll( 0.0 );
  jpos2mpos.SetAll( 0.0 );
  jtrq2mtrq.SetAll( 0.0 );
  
  mpos2jpos[0][0] = -0.0077437;
  mpos2jpos[1][1] = -0.0077437;
  mpos2jpos[2][2] = -0.0077437;
  mpos2jpos[3][3] = -0.057268;

  jpos2mpos[0][0] = 1.0 / mpos2jpos[0][0];
  jpos2mpos[1][1] = 1.0 / mpos2jpos[1][1]; 
  jpos2mpos[2][2] = 1.0 / mpos2jpos[2][2];
  jpos2mpos[3][3] = 1.0 / mpos2jpos[3][3];

  jtrq2mtrq[0][0] = -1.0;
  jtrq2mtrq[1][1] = -1.0; 
  jtrq2mtrq[2][2] = -1.0;
  jtrq2mtrq[3][3] = -1.0;

  //
  // Initialize the position of the BH8_280
  //
  
  for( size_t i=0; i<pucks.size(); i++ ){
    //pucks[i].SetProperty( Barrett::TIME2STOP, 250, true );
    //pucks[i].SetProperty( Barrett::MAXTRQ, 1500, true );
  }
  
  Hi();
  osaSleep( 1.0 );

  return osaBH8_280::ESUCCESS;

}

void osaBH8_280::Hi(){
  for( size_t i=0; i<pucks.size(); i++ )
    { pucks[i].SetProperty( Barrett::COMMAND, 13, false ); }
}

// set the motor positions 
osaBH8_280::Errno osaBH8_280::SetPositions( const vctDynamicVector<double>& jq ){

  // sanity check
  if( jq.size() != pucks.size() ){
    CMN_LOG_RUN_ERROR << "Expected " << pucks.size() << " joint angles. "
		      << "Got " << jq.size()
		      << std::endl;
    return osaBH8_280::EFAILURE;
  }

  // convert the joints positions to motor positions
  vctDynamicVector<double> mq = JointsPos2MotorsPos( jq );
  std::cout << mq << std::endl;
  // for each puck, send a position 
  for(size_t i=0; i<pucks.size(); i++){

    // Set the motor position
    if( pucks[i].SetPosition( mq[i] ) != osaPuck::ESUCCESS ){
      CMN_LOG_RUN_ERROR << "Failed to set pos of puck#: " 
			<< (int)pucks[i].GetID()
			<< std::endl;
    }

  }

  return osaBH8_280::ESUCCESS;
}


// query the joint positions
osaBH8_280::Errno osaBH8_280::GetPositions( vctDynamicVector<double>& jq ){

  vctDynamicVector<double> mq;
  if( handposition.GetPositions( mq ) != osaGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to get the upper arm positions"<<std::endl;
    return osaBH8_280::EFAILURE;
  }
  std::cout << mq << std::endl;
  jq = MotorsPos2JointsPos( mq );
  
  return osaBH8_280::ESUCCESS;

}

/*
osaBH8_280::Errno osaBH8_280::SetTorques( const vctDynamicVector<double>& jt ){

  switch( GetConfiguration() ){

  case osaBH8_280::BH8_280_4DOF:

    if( jt.size() != 4 ){
      vctDynamicVector<double> mt = JointsTrq2MotorsTrq( jt );

      vctFixedSizeVector<double,4> mtu( mt[0], mt[1], mt[2], mt[3] );
      if( uppertorques.SetTorques( mtu ) != osaGroup::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to set the upper arm torques" << std::endl;
	return osaBH8_280::EFAILURE;
      }
      
    }
    else{
      CMN_LOG_RUN_ERROR << "Expected 4 values. Got " << jt.size() << std::endl;
      return osaBH8_280::EFAILURE;
    }

    break;
    
  case osaBH8_280::BH8_280_7DOF:

    if( jt.size() == 7 ){

      vctDynamicVector<double> mt = JointsTrq2MotorsTrq( jt );
      
      vctFixedSizeVector<double,4> mtu( mt[0], mt[1], mt[2], mt[3] );
      if( uppertorques.SetTorques( mtu ) != osaGroup::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to set the upper arm torques" << std::endl;
	return osaBH8_280::EFAILURE;
      }
      
      vctFixedSizeVector<double,4> mtl( mt[4], mt[5], mt[6], 0.0 );
      if( lowertorques.SetTorques( mtl ) != osaGroup::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to set the lower arm torques" << std::endl;
	return osaBH8_280::EFAILURE;
      }

    }      
    else{
      CMN_LOG_RUN_ERROR << "Expected 7 values. Got " << jt.size() << std::endl;
      return osaBH8_280::EFAILURE;
    }

    break;

  }

  return osaBH8_280::ESUCCESS;

}
*/
vctDynamicVector<double> 
osaBH8_280::MotorsPos2JointsPos( const vctDynamicVector<double>& mq )
{  return mpos2jpos*mq;  }

vctDynamicVector<double> 
osaBH8_280::JointsPos2MotorsPos( const vctDynamicVector<double>& jq )
{  return jpos2mpos*jq;  }

vctDynamicVector<double> 
osaBH8_280::JointsTrq2MotorsTrq( const vctDynamicVector<double>& jt )
{  return jtrq2mtrq*jt;  }

