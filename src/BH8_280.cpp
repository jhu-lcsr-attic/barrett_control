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

#include <unistd.h>

#include <iostream>

#include <Eigen/Dense>

#include <barrett_direct/BH8_280.h>

using namespace barrett_direct;

// main constructor
BH8_280::BH8_280(	leoCAN::CANBus* canbus ) :
  // create the groups
  //broadcast(      Group::BROADCAST,         canbus, false ),
  hand(           Group::HAND,              canbus, false ),
  handposition(   Group::HAND_POSITION,      canbus ),

  qinit( qinit ) {

  //
  //canbus->AddFilter( leoCAN::CANBus::Filter( 0x051F, 0x0403 ) );

  // create the pucks
  pucks.push_back( Puck( Puck::PUCK_IDF1, canbus ) );
  pucks.push_back( Puck( Puck::PUCK_IDF2, canbus ) );
  pucks.push_back( Puck( Puck::PUCK_IDF3, canbus ) );
  pucks.push_back( Puck( Puck::PUCK_IDF4, canbus ) );


  if( canbus == NULL ) {
    std::cerr << "CAN device missing" << std::endl;
  }

}

BH8_280::~BH8_280(){}

BH8_280::Errno BH8_280::Initialize(){

  // initialize each puck
  for( size_t i=0; i<pucks.size(); i++ ){
    if( pucks[i].InitializeMotor() != Puck::ESUCCESS ){
      std::cerr << "Failed to initialize puck " << pucks[i].GetID()
			<< std::endl;
      return BH8_280::EFAILURE;
    }
  }

  /*
  // initialize the broadcast group
  if( broadcast.Initialize() != Group::ESUCCESS ){
    std::cerr << "Failed to initialize broadcast group"
		      << std::endl;
    return BH8_280::EFAILURE;
  }
  */

  // initialize the upper arm groups
  if( hand.Initialize() != Group::ESUCCESS ){
    std::cerr << "Failed to initialize hand group"
		      << std::endl;
    return BH8_280::EFAILURE;
  }
  
  if( handposition.Initialize() != Group::ESUCCESS ){
    std::cerr << "Failed to initialize hand positions group"
		      << std::endl;
    return BH8_280::EFAILURE;
  }

  
  // initialize the 4x4 transform matrices
  mpos2jpos.setZero( 4, 4 );
  jpos2mpos.setZero( 4, 4 );
  jtrq2mtrq.setZero( 4, 4 );
  
  mpos2jpos(0,0) = -0.0077437;
  mpos2jpos(1,1) = -0.0077437;
  mpos2jpos(2,2) = -0.0077437;
  mpos2jpos(3,3) = -0.057268;

  jpos2mpos(0,0) = 1.0 / mpos2jpos(0,0);
  jpos2mpos(1,1) = 1.0 / mpos2jpos(1,1); 
  jpos2mpos(2,2) = 1.0 / mpos2jpos(2,2);
  jpos2mpos(3,3) = 1.0 / mpos2jpos(3,3);

  jtrq2mtrq(0,0) = -1.0;
  jtrq2mtrq(1,1) = -1.0; 
  jtrq2mtrq(2,2) = -1.0;
  jtrq2mtrq(3,3) = -1.0;

  //
  // Initialize the position of the BH8_280
  //
  
  for( size_t i=0; i<pucks.size(); i++ ){
    //pucks[i].SetProperty( Barrett::TIME2STOP, 250, true );
    //pucks[i].SetProperty( Barrett::MAXTRQ, 1500, true );
  }
  
  Hi();
  usleep(1000000);

  return BH8_280::ESUCCESS;

}

void BH8_280::Hi(){
  for( size_t i=0; i<pucks.size(); i++ )
    { pucks[i].SetProperty( Barrett::COMMAND, 13, false ); }
}

// set the motor positions 
BH8_280::Errno BH8_280::SetPositions( const Eigen::VectorXd& jq ){

  // sanity check
  if( jq.size() != pucks.size() ){
    std::cerr << "Expected " << pucks.size() << " joint angles. "
		      << "Got " << jq.size()
		      << std::endl;
    return BH8_280::EFAILURE;
  }

  // convert the joints positions to motor positions
  Eigen::VectorXd mq = JointsPos2MotorsPos( jq );
  std::cout << mq << std::endl;
  // for each puck, send a position 
  for(size_t i=0; i<pucks.size(); i++){

    // Set the motor position
    if( pucks[i].SetPosition( mq[i] ) != Puck::ESUCCESS ){
      std::cerr << "Failed to set pos of puck#: " 
			<< (int)pucks[i].GetID()
			<< std::endl;
    }

  }

  return BH8_280::ESUCCESS;
}


// query the joint positions
BH8_280::Errno BH8_280::GetPositions( Eigen::VectorXd& jq ){

  Eigen::VectorXd mq;
  if( handposition.GetPositions( mq ) != Group::ESUCCESS ){
    std::cerr << "Failed to get the upper arm positions"<<std::endl;
    return BH8_280::EFAILURE;
  }
  std::cout << mq << std::endl;
  jq = MotorsPos2JointsPos( mq );
  
  return BH8_280::ESUCCESS;

}

/*
BH8_280::Errno BH8_280::SetTorques( const Eigen::VectorXd& jt ){

  switch( GetConfiguration() ){

  case BH8_280::BH8_280_4DOF:

    if( jt.size() != 4 ){
      Eigen::VectorXd mt = JointsTrq2MotorsTrq( jt );

      Eigen::Vector4d mtu( mt[0], mt[1], mt[2], mt[3] );
      if( uppertorques.SetTorques( mtu ) != Group::ESUCCESS ){
	std::cerr << "Failed to set the upper arm torques" << std::endl;
	return BH8_280::EFAILURE;
      }
      
    }
    else{
      std::cerr << "Expected 4 values. Got " << jt.size() << std::endl;
      return BH8_280::EFAILURE;
    }

    break;
    
  case BH8_280::BH8_280_7DOF:

    if( jt.size() == 7 ){

      Eigen::VectorXd mt = JointsTrq2MotorsTrq( jt );
      
      Eigen::Vector4d mtu( mt[0], mt[1], mt[2], mt[3] );
      if( uppertorques.SetTorques( mtu ) != Group::ESUCCESS ){
	std::cerr << "Failed to set the upper arm torques" << std::endl;
	return BH8_280::EFAILURE;
      }
      
      Eigen::Vector4d mtl( mt[4], mt[5], mt[6], 0.0 );
      if( lowertorques.SetTorques( mtl ) != Group::ESUCCESS ){
	std::cerr << "Failed to set the lower arm torques" << std::endl;
	return BH8_280::EFAILURE;
      }

    }      
    else{
      std::cerr << "Expected 7 values. Got " << jt.size() << std::endl;
      return BH8_280::EFAILURE;
    }

    break;

  }

  return BH8_280::ESUCCESS;

}
*/
Eigen::VectorXd 
BH8_280::MotorsPos2JointsPos( const Eigen::VectorXd& mq )
{  return mpos2jpos*mq;  }

Eigen::VectorXd 
BH8_280::JointsPos2MotorsPos( const Eigen::VectorXd& jq )
{  return jpos2mpos*jq;  }

Eigen::VectorXd 
BH8_280::JointsTrq2MotorsTrq( const Eigen::VectorXd& jt )
{  return jtrq2mtrq*jt;  }

