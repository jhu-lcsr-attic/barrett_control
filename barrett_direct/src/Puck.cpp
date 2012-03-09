/*

  Author(s): Simon Leonard
  Created on: Dec 02 2009

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

#include <barrett_direct/Puck.h>
#include <barrett_direct/Group.h>

using namespace barrett_direct;

Puck::ID operator++( Puck::ID& pid, int  ){
  switch(pid) {
    case Puck::PUCK_ID1:
    case Puck::PUCK_ID2:
    case Puck::PUCK_ID3:
    case Puck::PUCK_ID4:
    case Puck::PUCK_ID5:
    case Puck::PUCK_ID6:
    case Puck::PUCK_IDF1:
    case Puck::PUCK_IDF2:
    case Puck::PUCK_IDF3:
      return (Puck::ID)( pid + 1 );
    case Puck::PUCK_ID7:
    case Puck::SAFETY_MODULE_ID:
    case Puck::PUCK_IDF4:
      return Puck::SAFETY_MODULE_ID;
    default:
      std::cerr<<"WARNING: incrementing unknown Puck ID: "<<pid<<std::endl;
      return Puck::SAFETY_MODULE_ID;
  };
}

Puck::Puck(){}

// Initialize the puck to its ID and the CAN bus
Puck::Puck( Puck::ID id, leoCAN::CANBus* canbus, bool createfilter ){
  this->id = id;
  this->canbus = canbus;

  // Add a filter for the property feedback group (group 6)
  // 
  if( createfilter ){
    leoCAN::CANBusFrame::id_t filterid = 0x0000;
    filterid |= ( 0x00000001 << 10 ); // group bit shifted 10 bits 
    filterid |= ( GetID() << 5 );     // from  bits shifted 5 bits
    // G FFFFF TTTTT ( G:0-1, F:0-14, T: 0-14 )
    // GFF FFFT TTTT ( 0x5EF ) 
    canbus->AddFilter( leoCAN::CANBus::Filter( 0x05EF, (filterid | 0x0006) ) );
  }

}

std::string Puck::LogPrefix(){

  std::ostringstream oss;
  oss << "Puck " << (int)GetID() << ": ";
  return std::string( oss.str() );

}

// return the puck ID
Puck::ID Puck::GetID() const { return id; }  

// return the motor constant
Barrett::Value Puck::IpNm()                const { return ipnm; }

// return the encoder constant
Barrett::Value Puck::CountsPerRevolution() const { return cntprev; }

// return pucks index within its group
Barrett::Value Puck::GroupIndex()          const { return grpidx; }

// STATIC return the CAN ID of a message from the host (00000) to a puck ID
// A puck ID is represented by 5 bits whereas a CAN ID has 11
leoCAN::CANBusFrame::id_t Puck::CANID( Puck::ID id )
{ return (leoCAN::CANBusFrame::id_t)(0x1F & id); }

// STATIC return true if the CAN frame is a "set" command (a command has a 
// message of the form
// [1*** ****][**** ****]...[**** ****]
// thus we test if the MSB of the first byte is set
bool Puck::IsSetFrame( const leoCAN::CANBusFrame& canframe ){
  const leoCAN::CANBusFrame::data_t* data = canframe.GetData();
  return ( data[0] & Barrett::SET_CODE ) == Barrett::SET_CODE;
}

// STATIC returns the destination of a CAN id. This assumes that the 
// destination is a puck (as opposed to a group)
// The destination puck ID compose the 5 LSB of a CAN ID
Puck::ID Puck::DestinationID( leoCAN::CANBusFrame::id_t cid )
{ return (Puck::ID)( cid & 0x1F); }

// STATIC returns the destination of a CAN frame
Puck::ID Puck::DestinationID( const leoCAN::CANBusFrame& canframe )
{ return Puck::DestinationID( canframe.GetID() ); }

// STATIC returns the origin of a CAN id.
// the origin bits are the bits 5 to 9 (zero index) in a CAN ID
Puck::ID Puck::OriginID( leoCAN::CANBusFrame::id_t cid ) 
{ return (Puck::ID)((cid>>5) & 0x1F); }

// STATIC returns the origin of a CAN frame.
Puck::ID Puck::OriginID( const leoCAN::CANBusFrame& canframe ) 
{ return Puck::OriginID( canframe.GetID() ); }

// Get a property from the puck. this sends a query to the puck and wait for
// its reply
Puck::Errno Puck::GetProperty( Barrett::ID propid,
 				     Barrett::Value& propvalue ){ 

  // empty CAN frame
  leoCAN::CANBusFrame sendframe;
    
  // pack the query in a can frame
  if( PackProperty( sendframe, Barrett::GET, propid ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to pack property" << std::endl;
    return Puck::EFAILURE;
  }
  
  // send the CAN frame
  if( canbus->Send( sendframe ) != leoCAN::CANBus::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to querry puck" << std::endl;
    return Puck::EFAILURE;
  }
  
  // empty CAN frame
  leoCAN::CANBusFrame recvframe;

  // receive the response in a CAN frame
  if( canbus->Recv( recvframe ) != leoCAN::CANBus::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to receive property"
		      << std::endl;
    return Puck::EFAILURE;
  }
  
  // unpack the can frame
  Barrett::ID recvpropid;
  if(UnpackCANFrame( recvframe, recvpropid, propvalue ) != Puck::ESUCCESS){
    std::cerr << LogPrefix() << "Failed to unpack CAN frame."
		      << std::endl;
    return Puck::EFAILURE;
  }

  // make sure that the property received is the one we asked for
  if( propid != recvpropid ){
    std::cerr << LogPrefix() << "Oop! Unexpected property ID. "
		      << "Expected " << propid << " got " << recvpropid
		      << std::endl;
    return Puck::EFAILURE;
  }
  
  return Puck::ESUCCESS;
}

// this sets the property of a puck to a value
Puck::Errno Puck::SetProperty( Barrett::ID propid, 
				     Barrett::Value propval,
				     bool verify){

  // empty CAN frame
  leoCAN::CANBusFrame frame;

  // pack the property ID and value in a "set" CAN frame 
  if( PackProperty( frame, Barrett::SET, propid, propval )!=Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to pack property " << propid
			<< std::endl;
    return Puck::EFAILURE;
  }
  
  // send the CAN frame
  if( canbus->Send( frame ) != leoCAN::CANBus::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to send the CAN frame." 
		      << std::endl;
    return Puck::EFAILURE;
  }
  
  // do we double check that the value was set?
  if( verify ){
    
    // If we just changed the status of the puck, give it a bit of time to
    // initialize itself
    if( propid  == Barrett::STATUS && propval == Puck::STATUS_READY ) {
      usleep( 1000000 );
    } else {
      usleep( 10000 );
    }

    // query the puck to make sure that the property is set
    Barrett::Value recvpropval = rand();
    if( GetProperty( propid, recvpropval ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix()<<"Failed to get puck property"
			  << std::endl;
      return Puck::EFAILURE;
    }

    if( propval != recvpropval ){
      std::cerr << LogPrefix() << "Oop! Unexpected property value. " 
			  << "Expected " << propval << " got " << recvpropval
			  << std::endl;
      return Puck::EFAILURE;
    }

  }
  
  return Puck::ESUCCESS;
}

// This packs a frame originating from the host and destined to the puck
Puck::Errno Puck::PackProperty( leoCAN::CANBusFrame& canframe, 
				      Barrett::Command cmd,
				      Barrett::ID propid,
				      Barrett::Value propval ){

  // Can message is 8 bytes long
  leoCAN::CANBusFrame::data_field_t data={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  leoCAN::CANBusFrame::data_len_t length=1;  // default message length (for a query)
  
  // See Barrett's documentation to understand the format
  data[0] = propid & 0x7F;                                  // data[0] = APPPPPP
                                                            // data[1] = 0000000

  if(cmd == Barrett::SET){                                  // this is a 'SET'
    data[0] |= 0x80;                                        // data[0] = 1PPPPPP
    
    // fill the rest of the bytes with the property value
    for(size_t i=2; i<6; i++){
      data[i] = (leoCAN::CANBusFrame::data_t)( propval & 0xFF);     // data[i] = values
      propval >>= 8;
    }
    length = 6; // packed 6 bytes 
  }
  
  // create a new CAN frame
  canframe = leoCAN::CANBusFrame( Puck::CANID( GetID() ), data, length );

  return Puck::ESUCCESS;
}

// this unpacks a frame originating from the puck and destined to the host
// this is a bit backwards, because this methods is usually called from the 
// perspective of the host. Therefore, this is akin to asking a puck to unpack
// a message that it sent...whatever
Puck::Errno Puck::UnpackCANFrame(const leoCAN::CANBusFrame& canframe,
				       Barrett::ID& propid,
				       Barrett::Value& propval ){

  // get the data and the data length
  const leoCAN::CANBusFrame::data_t* data = canframe.GetData();
  leoCAN::CANBusFrame::data_len_t length = canframe.GetLength();

  // Ensure that the CAN frame originated from this puck!
  if( OriginID(canframe) == GetID() ){

    // Replies to position queries are ALWAYS "set" frames addressed to group 3
    if(IsSetFrame(canframe)                                   && // a SET frame?
       Group::IsDestinationAGroup(canframe)              && // to a group?
       Group::DestinationID(canframe) == Group::POSITION){// for #3?

      //std::cout << canframe << std::endl;

      // at this point we know that the CAN frame contain a motor position so
      // set the property ID to position
      propid = Barrett::POS;

      // decode the position payload
      propval = 0;
      propval |= ( (Barrett::Value)data[0] << 16) & 0x003F0000;
      propval |= ( (Barrett::Value)data[1] << 8 ) & 0x0000FF00;
      propval |= ( (Barrett::Value)data[2] )      & 0x000000FF;
      
      if(propval & 0x00200000)              // If negative 
	{ propval |= 0xffffffffFFC00000LL; }  // Sign-extend (64 bits)

      return Puck::ESUCCESS;     // done and done
    }
    // does the CAN frame contain a "set" command for the puck?
    if( IsSetFrame( canframe ) ){                  // is a "normal" SET?

      propid = (Barrett::ID)(data[0] & 0x7F);  // extract the property ID
      
      propval = 0;                                 // decode the payload
      leoCAN::CANBusFrame::data_len_t i;
      for(i=0; i<length-2; i++){
	propval |= ((Barrett::Value)data[i+2]<<(i*8)) & (0xFF<<(i*8));
      }

      if(propval & (1 << ((i*8) - 1)))             // Sign extend the value 
	{ propval |= 0xffffffffFFFFFFFFLL << (i*8); }// ???? Extend ????
      //{ propval |= 0xFFFFFFFF << (i*8); }
      
      return Puck::ESUCCESS;
    }
    
    // Assume firmware request (GET). Is this ever used?
    // This is remnants from old code
    propid = (Barrett::ID)(-(data[0] & 0x7F)); // extract the property ID
    propval = 0;                                   // set the propval
    
    return Puck::ESUCCESS;
  }

  std::cerr << LogPrefix() << "Frame ID = " << OriginID(canframe) 
		    << " does not match puck ID = "
		    << std::endl;
  
  return Puck::EFAILURE;
}

// configure the status of the puck and the motor/encoder constants
Puck::Errno Puck::InitializeMotor(){

  std::clog << LogPrefix() << "Initializing motor" << std::endl;

  Barrett::Value status;
  if( GetStatus( status ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to query the status" 
		      << std::endl;
    return Puck::EFAILURE;
  }
  
  // if the puck is "ready"
  if( status == Puck::STATUS_READY ){
   
    // set puck mode to idle
    if( SetMode( Puck::MODE_IDLE ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to idle" << std::endl;
      return Puck::EFAILURE;
    }

    // get the encoder constant
    if( GetCountsPerRev() != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to get Cnt/Rev" << std::endl;
      return Puck::EFAILURE;
    }

    // get motor constant
    if( GetIpNm() != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to get I/Nm" << std::endl;
      return Puck::EFAILURE;
    }
    
    // get group index
    if( GetGroupIndex() != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to get index" << std::endl;
      return Puck::EFAILURE;
    }
    
    // get group index
    if( GetMembership() != Puck::ESUCCESS ){
      std::cerr << LogPrefix() <<"Failed to get membership"<<std::endl;
      return Puck::EFAILURE;
    }
    
    return Puck::ESUCCESS;

  }

  // if the puck is not ready
  if( status == Puck::STATUS_RESET ){
    std::cerr << LogPrefix() << "Puck is resetting. Trying again." 
			<< std::endl;

    // change its status to ready
    if( SetProperty( Barrett::STATUS, Puck::STATUS_READY, true ) ){
      std::cerr << LogPrefix() << "Failed to wake up" << std::endl;
      return Puck::EFAILURE;
    }
    usleep(1000000);
    InitializeMotor();
  }

  return Puck::ESUCCESS;
}

Puck::Errno Puck::Reset(){
  if( SetProperty( Barrett::STATUS, Puck::STATUS_RESET, false ) != 
      Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to reset" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::Ready(){
  if( SetProperty( Barrett::STATUS, Puck::STATUS_READY, false ) != 
      Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to ready" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetPosition( Barrett::Value& position ){

  if( GetProperty( Barrett::POS, position ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to query position" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::InitializeSM(){

  if( GetID() == Puck::SAFETY_MODULE_ID ){

    //std::clog << LogPrefix() <<"Querying the status"<<std::endl;
    Barrett::Value smstatus;
    if( GetProperty( Barrett::STATUS, smstatus ) != SafetyModule::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to query the safety module."
			<< std::endl;
      return Puck::EFAILURE;
    }
    
    // check the safety module is "ready"
    if( smstatus != SafetyModule::STATUS_READY ){
      std::cerr << LogPrefix() << "The safety module is offline"
			<< std::endl;
      return Puck::EFAILURE;
    }  
    std::clog << LogPrefix() << "The safety module is online" 
			<< std::endl;
  
    // Set the velocity warning
    std::clog << LogPrefix() << "Setting velocity warning" 
			<< std::endl;
    if( SetVelocityWarning( 4000 ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "velocity warning not set"
			<< std::endl;
      return Puck::EFAILURE;
    }
    std::clog << LogPrefix() << "Velocity warning set" << std::endl;

    // Set the velocity fault
    std::clog << LogPrefix() << "Setting velocity fault" << std::endl;
    if( SetVelocityFault( 8000 ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "velocity fault not set"
			<< std::endl;
      return Puck::EFAILURE;
    }
    std::clog << LogPrefix() << "Velocity fault set" << std::endl;
    
    // Set the torque warning
    std::clog << LogPrefix() << "Setting torque warning" << std::endl;
    if( SetTorqueWarning( 4000 ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "torque warning not set"
			<< std::endl;
      return Puck::EFAILURE;
    }
    std::clog << LogPrefix() << "Torque warning set" << std::endl;

    // Set the torque fault
    std::clog << LogPrefix() << "Setting torque fault" << std::endl;
    if( SetTorqueFault( 8000 ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "torque fault not set"
			<< std::endl;
      return Puck::EFAILURE;
    }
    std::clog << LogPrefix() << "Torque fault set" << std::endl;

    std::clog << LogPrefix() << "The safety module is good to go" 
			<< std::endl;

    return Puck::ESUCCESS;
  }
  
  std::cerr << LogPrefix() << "Not a safety module" << std::endl;

  return Puck::EFAILURE;

}


Puck::Errno Puck::SetVelocityWarning( Barrett::Value vw ){

  if( GetID() == Puck::SAFETY_MODULE_ID ){
    if(SetProperty( Barrett::VELWARNING, vw, true )!=SafetyModule::ESUCCESS){
      std::cerr << LogPrefix() << "Unable to set the velocity warning." 
			<< std::endl;
      return Puck::EFAILURE;
    }
    return Puck::ESUCCESS;
  }
  
  std::cerr << LogPrefix() << "Not a safety module" << std::endl;

  return Puck::EFAILURE;

}

Puck::Errno Puck::SetVelocityFault( Barrett::Value vf ){

  if( GetID() == Puck::SAFETY_MODULE_ID ){
    if(SetProperty( Barrett::VELFAULT, vf, true ) != SafetyModule::ESUCCESS){
      std::cerr << LogPrefix() << "Unable to set the velocity fault" 
			<< std::endl;
      return Puck::EFAILURE;
    }
    return Puck::ESUCCESS;
  }

  std::cerr << LogPrefix() << "Not a safety module" << std::endl;

  return Puck::EFAILURE;

}

Puck::Errno Puck::SetTorqueWarning( Barrett::Value tw ){

  if( GetID() == Puck::SAFETY_MODULE_ID ){
    if(SetProperty( Barrett::TRQWARNING, tw, true )!=SafetyModule::ESUCCESS){
      std::cerr << LogPrefix() << "Unable to set the torques warning" 
			<< std::endl;
      return Puck::EFAILURE;
    }
    return Puck::ESUCCESS;
  }

  std::cerr << LogPrefix() << "Not a safety module" << std::endl;

  return Puck::EFAILURE;

}

Puck::Errno Puck::SetTorqueFault( Barrett::Value tf ){

  if( GetID() == Puck::SAFETY_MODULE_ID ){
    if(SetProperty( Barrett::TRQFAULT, tf, false )!= SafetyModule::ESUCCESS){
      std::cerr << LogPrefix() << "Unable to set the torques fault" 
			<< std::endl;
      return Puck::EFAILURE;
    }
    return Puck::ESUCCESS;
  }

  std::cerr << LogPrefix() << "Not a safety module" << std::endl;

  return Puck::EFAILURE;

}


Puck::Errno Puck::IgnoreFault( Barrett::Value fault ){

  if( GetID() == Puck::SAFETY_MODULE_ID ){
    if( SetProperty( Barrett::IGNOREFAULT, fault, true ) ){
      std::cerr << LogPrefix() << " Unable to set fault tolerance"
			<< std::endl;
      return Puck::EFAILURE;
    }
    return Puck::ESUCCESS;
  }
  
  std::cerr << LogPrefix() << "Not a safety module" << std::endl;

  return Puck::EFAILURE;

}

Puck::Errno Puck::SetPosition( double q ){

  Barrett::Value position;  // this is the position in encoder ticks
  
  // convert the motor positions to encoder ticks
  position = (Barrett::Value)floor( (q*CountsPerRevolution()) / (2.0*M_PI) );
  
  // Set the motor position. Don't double check the position because the 
  // noise might change the encoder.
  if( SetProperty( Barrett::POS, position, false ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to set pos of puck#: " 
		      << (int)GetID()
		      << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetStatus( Barrett::Value& status ){

  //std::clog << "GetStatus" << std::endl;
  if( GetProperty( Barrett::STATUS, status ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to query the status" 
		      << std::endl;
    return Puck::EFAILURE;
  }
  std::clog << LogPrefix() << "Status " << status << std::endl;

  return Puck::ESUCCESS;

}

Puck::Errno Puck::SetMode( Barrett::Value mode ){

  //std::clog << "SetMode: " << (int)mode << std::endl;
  // set puck mode
  if( SetProperty( Barrett::MODE, mode, true ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to set mode" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetMode( Barrett::Value& mode ){

  //std::clog << "SetMode: " << (int)mode << std::endl;
  // get puck mode
  if( GetProperty( Barrett::MODE, mode ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get mode" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetCountsPerRev(){

  // get the encoder constant
  if( GetProperty( Barrett::COUNTSPERREV, cntprev ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get resolution" <<std::endl;
    return Puck::EFAILURE;
  }
  std::clog << LogPrefix() << "Resolution: " << cntprev << std::endl;
  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetIpNm(){

  //std::clog << "GetIpNm" << std::endl;
  // get the motor torque constant
  if( GetProperty( Barrett::IPNM, ipnm ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get I/Nm" << std::endl;
    return Puck::EFAILURE;
  }
  std::clog << LogPrefix() << "I/Nm: " << ipnm << std::endl;

  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetGroupIndex(){

  //std::clog << "Get group index" << std::endl;
  // get the puck index
  if( GetProperty( Barrett::PUCKINDEX, grpidx ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get index" << std::endl;
    return Puck::EFAILURE;
  }
  std::clog << LogPrefix() << "Index: " << grpidx << std::endl;
  
  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetMembership(){

  //std::clog << "Get membership" << std::endl;

  if( GetGroupA() != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get group A" << std::endl;
    return Puck::EFAILURE;
  }
  
  if( GetGroupB() != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get group B" << std::endl;
    return Puck::EFAILURE;
  }

  if( GetGroupC() != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get group C" << std::endl;
    return Puck::EFAILURE;
  }

  if( GetID() == Puck::PUCK_ID5 || 
      GetID() == Puck::PUCK_ID6 || 
      GetID() == Puck::PUCK_ID7 ){

    if( groupC != 5 ){ 
      std::cerr << LogPrefix() << "Fixing membership of group C" 
			  << std::endl;
      
      if( SetGroupC( 5 ) != Puck::ESUCCESS ){
	std::cerr << LogPrefix() << "Failed to set group C" <<std::endl;
	return Puck::EFAILURE;
      }

      if( GetGroupC() != Puck::ESUCCESS ){
	std::cerr << LogPrefix() << "Failed to get group C" <<std::endl;
	return Puck::EFAILURE;
      }

      if( groupC == 5 )
	{ std::cerr << LogPrefix() << "Fix succeeded" << std::endl; }
      else
	{ std::cerr << LogPrefix() << "Fix failed" << std::endl; }
      
    }
    
  }

  std::clog << LogPrefix() 
		      << "Member of group:"
		      << " " << groupA
		      << " " << groupB
		      << " " << groupC
		      << std::endl;

  return Puck::ESUCCESS;

}

Puck::Errno Puck::GetGroupA(){

  //std::clog << "Get A membership" << std::endl;
  if( GetProperty( Barrett::GROUPA, groupA ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get group A" << std::endl;
    return Puck::EFAILURE;
  }
  
  return Puck::ESUCCESS;
}

Puck::Errno Puck::GetGroupB(){

  //std::clog << "Get B membership" << std::endl;
  if( GetProperty( Barrett::GROUPB, groupB ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get group B" << std::endl;
    return Puck::EFAILURE;
  }
  
  return Puck::ESUCCESS;
}

Puck::Errno Puck::GetGroupC(){

  //std::clog << "Get C membership" << std::endl;
  if( GetProperty( Barrett::GROUPC, groupC ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get group C" << std::endl;
    return Puck::EFAILURE;
  }
  
  return Puck::ESUCCESS;
}


Puck::Errno Puck::SetGroupA( Barrett::Value grp ){

  if( SetProperty( Barrett::GROUPA, grp, true ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to set group A" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::SetGroupB( Barrett::Value grp ){

  if( SetProperty( Barrett::GROUPB, grp, true ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to set group B" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}

Puck::Errno Puck::SetGroupC( Barrett::Value grp ){

  if( SetProperty( Barrett::GROUPC, grp, true ) != Puck::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to set group C" << std::endl;
    return Puck::EFAILURE;
  }

  return Puck::ESUCCESS;

}
