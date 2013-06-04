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

#include <iostream>

#include <Eigen/Dense>

#include <leo_can/CANBus.h>

#include <barrett_direct/WAM.h>

using namespace barrett_direct;

// main constructor
WAM::WAM(	leo_can::CANBus* canbus,
    WAM::Configuration configuration ) :

  configuration( configuration ),
  // create the groups
  broadcast(      Group::BROADCAST,         canbus ),
  uppertorques(   Group::UPPERARM,          canbus ),
  lowertorques(   Group::FOREARM,           canbus ),
  upperpositions( Group::UPPERARM_POSITION, canbus ),
  lowerpositions( Group::FOREARM_POSITION,  canbus ),

  // create the safety module
  safetymodule(   Puck::SAFETY_MODULE_ID,   canbus ),

  qinit() {


    // create the pucks
    pucks.push_back( Puck( Puck::PUCK_ID1, canbus ) );
    pucks.push_back( Puck( Puck::PUCK_ID2, canbus ) );
    pucks.push_back( Puck( Puck::PUCK_ID3, canbus ) );
    pucks.push_back( Puck( Puck::PUCK_ID4, canbus ) );

    if( GetConfiguration() == WAM::WAM_7DOF ){
      pucks.push_back( Puck( Puck::PUCK_ID5, canbus ) );
      pucks.push_back( Puck( Puck::PUCK_ID6, canbus ) );
      pucks.push_back( Puck( Puck::PUCK_ID7, canbus ) );
    }

    if( canbus == NULL )
    { std::cerr << "CAN device missing" << std::endl; }

  }

WAM::~WAM(){}

WAM::Errno WAM::Initialize(){

  // initialize the safety module
  if( safetymodule.InitializeSM() != Puck::ESUCCESS ){
    std::cerr << "Failed to initialize safety module"
      << std::endl;
    return WAM::EFAILURE;
  }

  if( safetymodule.IgnoreFault( 8 ) != Puck::ESUCCESS ){
    std::cerr << "Failed to configure the safety module" << std::endl;
    return WAM::EFAILURE;
  }

  // initialize each puck
  for( size_t i=0; i<pucks.size(); i++ ){
    if( pucks[i].InitializeMotor() != Puck::ESUCCESS ){
      std::cerr << "Failed to initialize puck " << pucks[i].GetID()
        << std::endl;
      return WAM::EFAILURE;
    }
  }


  // initialize the broadcast group
  if( broadcast.Initialize() != Group::ESUCCESS ){
    std::cerr << "Failed to initialize broadcast group"
      << std::endl;
    return WAM::EFAILURE;
  }

  // initialize the upper arm groups
  if( uppertorques.Initialize() != Group::ESUCCESS ){
    std::cerr << "Failed to initialize upper torques group"
      << std::endl;
    return WAM::EFAILURE;
  }

  if( upperpositions.Initialize() != Group::ESUCCESS ){
    std::cerr << "Failed to initialize upper positions group"
      << std::endl;
    return WAM::EFAILURE;
  }


  switch( GetConfiguration() ){

    case WAM::WAM_4DOF:

      // initialize the 4x4 transform matrices
      mpos2jpos.setZero( 4, 4 );
      jpos2mpos.setZero( 4, 4 );
      jtrq2mtrq.setZero( 4, 4 );

      mpos2jpos(0,0) = -0.0238095;
      mpos2jpos(1,1) =  0.0176991;   mpos2jpos(1,2) = -0.0176991;
      mpos2jpos(2,1) = -0.0297345;   mpos2jpos(2,2) = -0.0297345;
      mpos2jpos(3,3) = -0.0555556;

      jpos2mpos(0,0) = -42.0;
      jpos2mpos(1,1) =  28.25;       jpos2mpos(1,2) = -16.8155;
      jpos2mpos(2,1) = -28.25;       jpos2mpos(2,2) = -16.8155;
      jpos2mpos(3,3) = -18.0;

      jtrq2mtrq(0,0) = -0.0238095;
      jtrq2mtrq(1,1) =  0.0176991;   jtrq2mtrq(1,2) = -0.0297345;
      jtrq2mtrq(2,1) = -0.0176991;   jtrq2mtrq(2,2) = -0.0297345;
      jtrq2mtrq(3,3) = -0.0555556;

      break;

    case WAM::WAM_7DOF:

      // for 7 dof initialize the lower arm groups
      if( lowertorques.Initialize() != Group::ESUCCESS ){
        std::cerr << "Failed to initialize lower torques group"
          << std::endl;
        return WAM::EFAILURE;
      }

      if( lowerpositions.Initialize() != Group::ESUCCESS ){
        std::cerr << "Failed to initialize lower positions group"
          << std::endl;
        return WAM::EFAILURE;
      }

      // initialize the 7x7 transform matrices
      mpos2jpos.setZero( 7, 7 );
      jpos2mpos.setZero( 7, 7 );
      jtrq2mtrq.setZero( 7, 7 );

      /*
      mpos2jpos << -0.0238095,        0.0,        0.0,        0.0,        0.0,        0.0,        0.0,
                          0.0,  0.0176991, -0.0176991,        0.0,        0.0,        0.0,        0.0,
                          0.0, -0.0297345, -0.0297345,        0.0,        0.0,        0.0,        0.0,
                          0.0,        0.0,        0.0, -0.0555556,        0.0,        0.0,        0.0,
                          0.0,        0.0,        0.0,        0.0,  0.0527426,  0.0527426,        0.0,
                          0.0,        0.0,        0.0,        0.0, -0.0527426,  0.0527426,        0.0,
                          0.0,        0.0,        0.0,        0.0,        0.0,        0.0, -0.0669792;
      */
      mpos2jpos(0,0) = -0.0238095;
      mpos2jpos(1,1) =  0.0176991;   mpos2jpos(1,2) = -0.0176991;
      mpos2jpos(2,1) = -0.0297345;   mpos2jpos(2,2) = -0.0297345;
      mpos2jpos(3,3) = -0.0555556;

      jpos2mpos(0,0) = -42.0;
      jpos2mpos(1,1) =  28.25;       jpos2mpos(1,2) = -16.8155;
      jpos2mpos(2,1) = -28.25;       jpos2mpos(2,2) = -16.8155;
      jpos2mpos(3,3) = -18.0;

      jtrq2mtrq(0,0) = -0.0238095;
      jtrq2mtrq(1,1) =  0.0176991;   jtrq2mtrq(1,2) = -0.0297345;
      jtrq2mtrq(2,1) = -0.0176991;   jtrq2mtrq(2,2) = -0.0297345;
      jtrq2mtrq(3,3) = -0.0555556;

      mpos2jpos(4,4) =  0.0527426; mpos2jpos(4,5) = 0.0527426;
      mpos2jpos(5,4) = -0.0527426; mpos2jpos(5,5) = 0.0527426;
      mpos2jpos(6,6) = -0.0669792;

      jpos2mpos(4,4) =   9.48;     jpos2mpos(4,5) = -9.48;
      jpos2mpos(5,4) =   9.48;     jpos2mpos(5,5) =  9.48;
      jpos2mpos(6,6) = -14.93;

      jtrq2mtrq(4,4) =  0.0527426; jtrq2mtrq(4,5) = -0.0527426;
      jtrq2mtrq(5,4) =  0.0527426; jtrq2mtrq(5,5) =  0.0527426;
      jtrq2mtrq(6,6) = -0.0669792;

      break;

  }

  return WAM::ESUCCESS;

}

WAM::Errno WAM::SetVelocityWarning( Barrett::Value vw ){

  if( safetymodule.SetVelocityWarning( vw ) != SafetyModule::ESUCCESS ){
    std::cerr << "Unable to set the velocity warning" << std::endl;
    return WAM::EFAILURE;
  }

  return WAM::ESUCCESS;

}

WAM::Errno WAM::SetVelocityFault( Barrett::Value vf ){

  if( safetymodule.SetVelocityFault( vf ) != SafetyModule::ESUCCESS ){
    std::cerr << "Unable to set the velocity fault" << std::endl;
    return WAM::EFAILURE;
  }

  return WAM::ESUCCESS;
}

WAM::Errno WAM::SetTorqueWarning( Barrett::Value tw ){

  if( safetymodule.SetTorqueWarning( tw ) != SafetyModule::ESUCCESS ){
    std::cerr << "Unable to set the torques warning" << std::endl;
    return WAM::EFAILURE;
  }
  return WAM::ESUCCESS;
}

WAM::Errno WAM::SetTorqueFault( Barrett::Value tf ){

  if( safetymodule.SetTorqueFault( tf ) != SafetyModule::ESUCCESS ){
    std::cerr << "Unable to set the torques fault" << std::endl;
    return WAM::EFAILURE;
  }
  return WAM::ESUCCESS;
}

WAM::Errno WAM::SetMode( Barrett::Value mode ){

  if( broadcast.SetMode( mode ) != Group::ESUCCESS ){
    std::cerr << "Failed to set mode" << std::endl;
    return WAM::EFAILURE;
  }
  return WAM::ESUCCESS;

}

WAM::Errno WAM::GetMode( WAM::Mode& mode ){
  mode = WAM::MODE_ACTIVATED;

  for( size_t i=0; i<pucks.size(); i++ ){
    Barrett::Value mi = Puck::MODE_IDLE;

    if( pucks[i].GetMode( mi ) != Puck::ESUCCESS ){
      std::cerr << "Failed to get mode" << std::endl;
      return WAM::EFAILURE;
    }

    if( mi == Puck::MODE_IDLE ){
      mode = WAM::MODE_IDLE;
      return WAM::ESUCCESS;
    }

  }

  return WAM::ESUCCESS;

}


const Barrett::Value WAM::MAX_COUNTS;

WAM::Errno WAM::GetResolverRanges( Eigen::VectorXd& resolver_ranges ) {
  Eigen::VectorXd mq = Eigen::VectorXd::Constant(WAM::DOF(configuration), 2.0*M_PI);
  resolver_ranges = (mpos2jpos.diagonal().array() * mq.array()).cwiseAbs();

  return WAM::ESUCCESS;
}

#if 0
WAM::Errno WAM::GetJointToActuator( size_t i, size_t j, Eigen::Matrix4d& jta ) {
  jta << jpos2mpos(i,i), jpos2mpos(i,j),
          jpos2mpos(j,i), jpos2mpos(j,j);

  return WAM::ESUCCESS;
}
#endif

WAM::Errno WAM::GetPositionOffsets( Eigen::VectorXd& jq ) {
  // TODO: Servo the updated calibration position so to not violate the
  // velocity constraints

  // Initialize output
  Eigen::VectorXd mq = Eigen::VectorXd::Zero(WAM::DOF(configuration));

  for(size_t i=0; i<pucks.size(); i++){
    Barrett::Value count;

    // Get the magnetic absolute encoder reading
    if( pucks[i].GetProperty( Barrett::MECHANGLE, count) != Puck::ESUCCESS ) {
      std::cerr << "Failed to get megnetic encoder reading of puck: "
        << (int)pucks[i].GetID() << std::endl;
      return WAM::EFAILURE;
    } 

    // Calculate the error magnitude
    //mag_error[i] = mag_zero[i] - count;

    //if( std::abs(mag_error[i]) > MAX_COUNTS/2 ) {
    //  mag_error[i] -= std::copysign(1.0, mag_error)*MAX_COUNTS;
    //} 

    // Convert to angle
    mq[i] = count * M_PI * 2.0 / (double)MAX_COUNTS;

  }

  // Get the joint offsets from the motor positions
  jq = MotorsPos2JointsPos(mq);

  return WAM::ESUCCESS;
}

// set the motor positions 
WAM::Errno WAM::SetPositions( const Eigen::VectorXd& jq ){

  // sanity check
  if( (size_t)jq.size() != pucks.size() ){
    std::cerr << "Expected " << pucks.size() << " joint angles. "
      << "Got " << jq.size()
      << std::endl;
    return WAM::EFAILURE;
  }

  // let the safety module ignore a few faults
  // this is necessary because otherwise the safety module will monitor a large
  // change of joint position in a short amount of time and trigger a velocity 
  // fault.  
  if( safetymodule.IgnoreFault( 8 ) != Puck::ESUCCESS ){
    std::cerr << "Failed to configure the safety module" << std::endl;
    return WAM::EFAILURE;
  }

  // convert the joints positions to motor positions
  Eigen::VectorXd mq = JointsPos2MotorsPos( jq );


  // for each puck, send a position 
  for(size_t i=0; i<pucks.size(); i++){

    // Set the motor position
    if( pucks[i].SetPosition( mq[i] ) != Puck::ESUCCESS ){
      std::cerr << "Failed to set pos of puck#: " 
        << (int)pucks[i].GetID()
        << std::endl;
    }
    usleep(1000);
  }

  // let the safety module ignore a few faults
  // this is necessary because otherwise the safety module will monitor a large
  // change of joint position in a short amount of time and trigger a velocity 
  // fault.  
  if( safetymodule.IgnoreFault( 1 ) != Puck::ESUCCESS ){
    std::cerr << "Failed to configure the safety module" << std::endl;
    return WAM::EFAILURE;
  }

  return WAM::ESUCCESS;
}


// query the joint positions
WAM::Errno WAM::GetPositions( Eigen::VectorXd& jq ){

  switch( GetConfiguration() ){

    case WAM::WAM_4DOF:

      {
        Eigen::VectorXd mq;
        if( upperpositions.GetPositions( mq ) != Group::ESUCCESS ){
          std::cerr << "Failed to get the upper arm positions"<<std::endl;
          return WAM::EFAILURE;
        }
        jq = MotorsPos2JointsPos( mq );
      }

      break;

    case WAM::WAM_7DOF:

      {
        Eigen::VectorXd mqu, mql;

        if( upperpositions.GetPositions( mqu ) != Group::ESUCCESS ){
          std::cerr << "Failed to get the upper arm positions"<<std::endl;
          return WAM::EFAILURE;
        }

        if( lowerpositions.GetPositions( mql ) != Group::ESUCCESS ){
          std::cerr << "Failed to get the lower arm positions"<<std::endl;
          return WAM::EFAILURE;
        }

        Eigen::VectorXd mq(7);
        mq << mqu[0], mqu[1], mqu[2], mqu[3], 
           mql[0], mql[1], mql[2];

        jq = MotorsPos2JointsPos( mq );

      }

      break;

  }

  return WAM::ESUCCESS;

}

WAM::Errno WAM::SetTorques( const Eigen::VectorXd& jt ){

  switch( GetConfiguration() ){

    case WAM::WAM_4DOF:

      if( jt.size() != 4 ){
        Eigen::VectorXd mt = JointsTrq2MotorsTrq( jt );

        Eigen::Vector4d mtu( mt[0], mt[1], mt[2], mt[3] );
        if( uppertorques.SetTorques( mtu ) != Group::ESUCCESS ){
          std::cerr << "Failed to set the upper arm torques" << std::endl;
          return WAM::EFAILURE;
        }

      }
      else{
        std::cerr << "Expected 4 values. Got " << jt.size() << std::endl;
        return WAM::EFAILURE;
      }

      break;

    case WAM::WAM_7DOF:

      if( jt.size() == 7 ){

        Eigen::VectorXd mt = JointsTrq2MotorsTrq( jt );

        Eigen::Vector4d mtu( mt[0], mt[1], mt[2], mt[3] );
        if( uppertorques.SetTorques( mtu ) != Group::ESUCCESS ){
          std::cerr << "Failed to set the upper arm torques" << std::endl;
          return WAM::EFAILURE;
        }

        Eigen::Vector4d mtl( mt[4], mt[5], mt[6], 0.0 );
        if( lowertorques.SetTorques( mtl ) != Group::ESUCCESS ){
          std::cerr << "Failed to set the lower arm torques" << std::endl;
          return WAM::EFAILURE;
        }

      }      
      else{
        std::cerr << "Expected 7 values. Got " << jt.size() << std::endl;
        return WAM::EFAILURE;
      }

      break;

  }

  return WAM::ESUCCESS;

}

  Eigen::VectorXd 
WAM::MotorsPos2JointsPos( const Eigen::VectorXd& mq )
{  return mpos2jpos*mq;  }

  Eigen::VectorXd 
WAM::JointsPos2MotorsPos( const Eigen::VectorXd& jq )
{  return jpos2mpos*jq;  }

  Eigen::VectorXd 
WAM::JointsTrq2MotorsTrq( const Eigen::VectorXd& jt )
{  return jtrq2mtrq*jt;  }

