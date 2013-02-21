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

#include <iostream>

#include <Eigen/Dense>

#include <leoCAN/CANBus.h>

#include <barrett_direct/Group.h>

using namespace barrett_direct;

Group::ID operator++( Group::ID& gid, int ){

  if( gid==Group::BROADCAST )
  { return gid = Group::UPPERARM; }

  if( gid==Group::UPPERARM )
  { return gid = Group::FOREARM; }

  if( gid==Group::FOREARM )
  { return gid = Group::POSITION; }

  if( gid==Group::POSITION )
  { return gid = Group::UPPERARM_POSITION; }

  if( gid==Group::UPPERARM_POSITION ) 
  { return gid = Group::FOREARM_POSITION; }

  if( gid==Group::FOREARM_POSITION )
  { return gid = Group::PROPERTY; }

  if( gid==Group::PROPERTY ) 
  { return gid = Group::HAND; }

  if( gid==Group::HAND )
  { return gid = Group::HAND_POSITION; }

  if( gid==Group::HAND_POSITION )
  { return gid = Group::LASTGROUP; }

  return gid = Group::LASTGROUP;

}

// default constructor
Group::Group( Group::ID id, leoCAN::CANBus* canbus, bool createfilter ) : 
  canbus( canbus ),
  id( id ){

    switch( GetID() ){

      case Group::BROADCAST:
        AddPuckToGroup( Puck::PUCK_ID1 );
        AddPuckToGroup( Puck::PUCK_ID2 );
        AddPuckToGroup( Puck::PUCK_ID3 );
        AddPuckToGroup( Puck::PUCK_ID4 );
        AddPuckToGroup( Puck::PUCK_ID5 );
        AddPuckToGroup( Puck::PUCK_ID6 );
        AddPuckToGroup( Puck::PUCK_ID7 );

        if( createfilter ){
        }

        break;

        // used to set torques
      case Group::UPPERARM:
        AddPuckToGroup( Puck::PUCK_ID1 );
        AddPuckToGroup( Puck::PUCK_ID2 );
        AddPuckToGroup( Puck::PUCK_ID3 );
        AddPuckToGroup( Puck::PUCK_ID4 );
        break;

        // used to set torques
      case Group::FOREARM:
        AddPuckToGroup( Puck::PUCK_ID5 );
        AddPuckToGroup( Puck::PUCK_ID6 );
        AddPuckToGroup( Puck::PUCK_ID7 );
        break;

      case Group::POSITION:
        AddPuckToGroup( Puck::PUCK_IDF1 );
        AddPuckToGroup( Puck::PUCK_IDF2 );
        AddPuckToGroup( Puck::PUCK_IDF3 );
        AddPuckToGroup( Puck::PUCK_IDF4 );
        break;

        // used to get positions
      case Group::UPPERARM_POSITION:
        AddPuckToGroup( Puck::PUCK_ID1 );
        AddPuckToGroup( Puck::PUCK_ID2 );
        AddPuckToGroup( Puck::PUCK_ID3 );
        AddPuckToGroup( Puck::PUCK_ID4 );
        if( createfilter ){
          // G FFFFF TTTTT ( G:0-1, F:0-14, T: 0-14 )
          // GFF FFFT TTTT ( 0x5EF ) 
          // Only fiter position to group 3. So G is set and TTTTT = 00011
          // Also, the pucks id for the upper arm have FFFFF=00xxx such that we have
          // a mask: 0x05E3 and filter: 0x0423, 0x0443, 0x0463, 0x0483. Here we 
          // filter all 4 pucks because the forearm has similar filters
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x0423 ) );
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x0443 ) );
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x0463 ) );
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x0483 ) );
        }

        break;

        // used to get positions
      case Group::FOREARM_POSITION:
        AddPuckToGroup( Puck::PUCK_ID5 );
        AddPuckToGroup( Puck::PUCK_ID6 );
        AddPuckToGroup( Puck::PUCK_ID7 );
        if( createfilter ){
          // G FFFFF TTTTT ( G:0-1, F:0-14, T: 0-14 )
          // GFF FFFT TTTT ( 0x5EF ) 
          // Only fiter position to group 3. So G is set and TTTTT = 00011
          // Also, the pucks id for the upper arm have FFFFF=00xxx such that we have
          // a mask: 0x05E3 and filter: 0x0423, 0x0443, 0x0463, 0x0483. Here we 
          // filter all 4 pucks because the forearm has similar filters
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x04A3 ) );
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x04C3 ) );
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x05E3, 0x04E3 ) );
        }

        break;

      case Group::PROPERTY:
        break;

      case Group::HAND:
        AddPuckToGroup( Puck::PUCK_IDF1 );
        AddPuckToGroup( Puck::PUCK_IDF2 );
        AddPuckToGroup( Puck::PUCK_IDF3 );
        AddPuckToGroup( Puck::PUCK_IDF4 );
        break;

      case Group::HAND_POSITION:
        AddPuckToGroup( Puck::PUCK_IDF1 );
        AddPuckToGroup( Puck::PUCK_IDF2 );
        AddPuckToGroup( Puck::PUCK_IDF3 );
        AddPuckToGroup( Puck::PUCK_IDF4 );
        if( createfilter ){
          // G FFFFF TTTTT ( G:0-1, F:0-14, T: 0-14 )
          // GFF FFFT TTTT ( 0x5EF ) 
          // Only fiter position to group 3. So G is set and TTTTT = 00011
          // Also, the pucks id for the hand have FFFFF=01xxx such that we have:
          // a mask: 0x0503 and id: 0x0503. The hand is the only device with the 
          // 4th bit to 1 so use that bit to determine if the hand is replying
          canbus->AddFilter( leoCAN::CANBus::Filter( 0x0503, 0x0503 ) );
        }

        break;

      default:
        break;
    }

  }

std::string Group::LogPrefix(){
  std::string s;
  switch( GetID() ){
    case BROADCAST:
      s = std::string( "BROADCAST" );
      break;

    case UPPERARM:
      s = std::string( "UPPERARM" );
      break;

    case FOREARM:
      s = std::string( "FOREARM" );
      break;

    case POSITION:
      s = std::string( "POSITION" );
      break;

    case UPPERARM_POSITION:
      s = std::string( "UPPERARM_POSITION" );
      break;

    case FOREARM_POSITION:
      s = std::string( "FOREARM_POSITION" );
      break;

    case PROPERTY:
      s = std::string( "PROPERTY" );
      break;

    case HAND:
      s = std::string( "HAND" );
      break;

    case HAND_POSITION:
      s = std::string( "HAND_POSITION" );
      break;

    default:
      s = std::string( "UNKNOWN" );

  }

  std::ostringstream oss;
  oss << "Group " << s << ": ";
  return std::string( oss.str() );

}

// STATIC return true of the CAN frame is destined to a group
// For this we test the CAN ID for the GROUPTAG bit
bool Group::IsDestinationAGroup( const leoCAN::CANBusFrame canframe )
{ return (canframe.GetID() & Group::GROUP_CODE) == Group::GROUP_CODE; }

// return the group id
Group::ID Group::GetID() const { return id; }

// STATIC return the CAN of a message from the host (00000) to a group ID
// A group ID is represented by 5 bits whereas a CAN ID has 11
leoCAN::CANBusFrame::id_t Group::CANID( Group::ID id )
{ return (leoCAN::CANBusFrame::id_t)( Group::GROUP_CODE | (0x1F & id) ); }

// STATIC return true if the CAN frame is a "set" command (a command has a 
// message of the form
// [1*** ****][**** ****]...[**** ****]
// thus we test if the MSB of the first byte is set
bool Group::IsSetFrame( const leoCAN::CANBusFrame& canframe ){
  const leoCAN::CANBusFrame::data_t* data = canframe.GetData();
  return ( data[0] & Barrett::SET_CODE) == Barrett::SET_CODE;
}

// STATIC returns the destination of a CAN id. This assumes that the 
// destination is a group (as opposed to a puck)
// The destination group ID compose the 5 LSB of a CAN ID
Group::ID Group::DestinationID( leoCAN::CANBusFrame::id_t cid )
{ return (Group::ID) ( cid & 0x001F); }

// STATIC extract the destination group ID
Group::ID Group::DestinationID( const leoCAN::CANBusFrame& canframe )
{ return Group::DestinationID( canframe.GetID() ); }

// STATIC extract the origin group ID
// the origin bits are the bits 5 to 9 (zero index) in a CAN ID
Group::ID Group::OriginID( leoCAN::CANBusFrame::id_t cid )
{ return (Group::ID) ((cid>>5) & 0x001F); }

// STATIC extract the origin group ID
Group::ID Group::OriginID( const leoCAN::CANBusFrame& canframe )
{ return Group::OriginID( canframe.GetID() ); }

void Group::AddPuckToGroup( Puck::ID pid ){

  Puck puck( pid, canbus, false );
  pucks.push_back( puck );

}

// Query a group of puck.
Group::Errno Group::GetProperty( Barrett::ID propid, 
    std::vector<Barrett::Value>& values ){

  // pack the query in a CAN frame
  leoCAN::CANBusFrame sendframe;
  if( PackProperty( sendframe, Barrett::GET, propid ) != Group::ESUCCESS){
    std::cerr << "Failed to pack the property" << std::endl;
    return Group::EFAILURE;
  }

  // send the CAN frame
  if( canbus->Send( sendframe ) != leoCAN::CANBus::ESUCCESS ){
    std::cerr << ": Failed to querry group" << std::endl;
    return Group::EFAILURE;
  }

  values.clear();
  values.resize( pucks.size() );

  for( size_t i=0; i<pucks.size(); i++ ){

    // empty CAN frame
    leoCAN::CANBusFrame recvframe;

    // receive the response in a CAN frame
    if( canbus->Recv( recvframe ) != leoCAN::CANBus::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to receive property" 
        << std::endl;
      return Group::EFAILURE;
    }

    
    //std::cerr << recvframe << std::endl<<std::endl;

    // figure which puck send that frame
    Puck::ID pid = Puck::OriginID( recvframe );
    int pindex = -1;
    for( size_t j=0; j<pucks.size(); j++ ){
      if( pucks[j].GetID() == pid )
      { pindex = j; }
    }

    // unpack the frame
    if( -1 < pindex ){

      Barrett::ID recvpropid;
      Barrett::Value recvvalue;

      // unpack the frame;
      if( pucks[pindex].UnpackCANFrame( recvframe, recvpropid, recvvalue ) 
          != Puck::ESUCCESS){
        std::cerr << LogPrefix() << "Failed to unpack CAN frame"
          << std::endl;
        return Group::EFAILURE;
      }

      // make sure that the property received is the one we asked for
      if( propid != recvpropid ){
        std::cerr << LogPrefix() << "Unexpected property ID. "
          << "Expected " << propid << " got " << recvpropid
          << std::endl;
        return Group::EFAILURE;
      }

      if( 0 <= pindex && pindex < (int)values.size() )
      { values[ pindex ] = recvvalue; }
      else
      { std::cerr << LogPrefix() << "Could not index the value vector"
        << std::endl; }
    }
    else{
      std::cerr << LogPrefix() << "Could not index the pucks" 
        << std::endl;
    }

  }

  return Group::ESUCCESS;

}


// Set the properties of a group
// Unlike devPuck::SetProperty, this doesn't verify the pucks values
Group::Errno Group::SetProperty( Barrett::ID propid, 
    Barrett::Value propval,
    bool verify){

  // pack the "set" command in a CAN frame
  leoCAN::CANBusFrame canframe;
  if( PackProperty( canframe, Barrett::SET, propid, propval )
      != Group::ESUCCESS ){
    std::cerr << ": Failed to pack the property " << propid 
      << std::endl;
    return Group::EFAILURE;
  }

  // Send the CAN frame
  if( canbus->Send( canframe ) != leoCAN::CANBus::ESUCCESS ){
    std::cerr << ": Failed to send the CAN frame."
      << std::endl;
    return Group::EFAILURE;
  }

  if( verify ){
    std::cerr << __FILE__
      << ": Verify is not implemented for groups."
      << std::endl;
  }

  return Group::ESUCCESS;

}

// This packs a frame originating from the host and destined to the puck
Group::Errno Group::PackProperty( leoCAN::CANBusFrame& canframe,
    Barrett::Command cmd,
    Barrett::ID propid,
    Barrett::Value propval ){

  leoCAN::CANBusFrame::data_field_t data={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  leoCAN::CANBusFrame::data_len_t length=1;  // default message length (for a query)

  // See Barrett's documentation to understand the format
  data[0] = propid & 0x7F;                                 //data[0] = APPPPPP
  data[1] = 0x00;                                          //data[1] = 0000000

  if(cmd == Barrett::SET){                                  //this is a 'SET'
    data[0] |= 0x80;                                        //data[0] = 1PPPPPP

    // fill the rest of the bytes with the property value
    for(size_t i=2; i<6; i++){
      data[i] = (leoCAN::CANBusFrame::data_t)(propval & 0x000000FF);//data[i] = values
      propval >>= 8;
    }
    length = 6; // packed 6 bytes 
  }

  // create a new CAN frame
  canframe = leoCAN::CANBusFrame( Group::CANID( GetID() ), data, length );

  return Group::ESUCCESS;
}


Group::Errno Group::Reset(){
  if( SetProperty( Barrett::STATUS, Puck::STATUS_RESET, false ) != 
      Group::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to reset" << std::endl;
    return Group::EFAILURE;
  }

  return Group::ESUCCESS;

}


Group::Errno Group::Ready(){
  if( SetProperty( Barrett::STATUS, Puck::STATUS_READY, false ) != 
      Group::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to ready" << std::endl;
    return Group::EFAILURE;
  }

  return Group::ESUCCESS;

}

Group::Errno Group::GetPositions( Eigen::VectorXd& q ){

  // Query a group of puck.
  std::vector<Barrett::Value> values;

  if( GetProperty( Barrett::POS, values ) != Group::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to get positions" << std::endl;
    return Group::EFAILURE;
  }

  if( values.size() == pucks.size() ){

    q.resize( pucks.size() );

    for( size_t i=0; i<pucks.size(); i++ ){

      // convert the position from encoder ticks to radians
      q[i] = ( ((double)values[i]) * 2.0 * M_PI  /
          ((double)pucks[i].CountsPerRevolution() ) );

    }

  }
  else{
    std::cerr << LogPrefix() << "Expected " << pucks.size() <<" values."
      << " Got " << values.size() 
      << std::endl;
    return Group::EFAILURE;    
  }

  return Group::ESUCCESS;

}


Group::Errno Group::SetTorques( const Eigen::Vector4d& tau ){

  if( GetID() == Group::UPPERARM || GetID() == Group::FOREARM ){

    Eigen::Vector4d currents(tau.size());
    for( int i=0; i<currents.size(); i++ )
    { currents[i] = tau[i] * pucks[i].IpNm(); }

    // pack the torques in a can frames
    leoCAN::CANBusFrame frame;
    if( PackCurrents( frame, currents ) != Group::ESUCCESS ){
      std::cerr << "Failed to pack the torques" << std::endl;
      return Group::EFAILURE;
    }

    // send the canframe (blocking)
    if( canbus->Send( frame ) != leoCAN::CANBus::ESUCCESS ){
      std::cerr << "Failed to send upper arm torques" << std::endl;
      return Group::EFAILURE;
    }

  }
  else{
    std::cerr << LogPrefix() 
      << "Group  cannot send torques" << std::endl;
    return Group::EFAILURE;
  }

  return Group::ESUCCESS;

}


// pack motor torques in a CAN frame
// this should go into devGroup
Group::Errno Group::PackCurrents( leoCAN::CANBusFrame& frame, 
    const Eigen::Vector4d& I ){

  // we can only pack torques for the upper arm and forearm groups
  if( GetID() == Group::UPPERARM || GetID() == Group::FOREARM ){

    // copy each motor torques in this array with the correct index
    Barrett::Value values[4] = {0, 0, 0, 0};

    // for each puck in the group
    for( size_t i=0; i<pucks.size(); i++ ){

      // get the index of the puck within its group [0,1,2,3]
      int idx =  pucks[i].GroupIndex()-1;          // -1 because of zero index
      if( idx < 0 || 3 < idx ){                    // sanity check
        std::cerr << "Illegal index" << std::endl;
        return Group::EFAILURE;
      }

      values[ idx ] = (Barrett::Value)I[i];        // cast the torque      
    }

    // pack the torques in a 8 bytes message (see the documentation)
    unsigned char msg[8];
    msg[0]= Barrett::TRQ | 0x80;
    msg[1]=(unsigned char)(( values[0]>>6)&0x00FF);
    msg[2]=(unsigned char)(((values[0]<<2)&0x00FC)|((values[1]>>12)&0x0003));
    msg[3]=(unsigned char)(( values[1]>>4)&0x00FF);
    msg[4]=(unsigned char)(((values[1]<<4)&0x00F0)|((values[2]>>10)&0x000F));
    msg[5]=(unsigned char)(( values[2]>>2)&0x00FF);
    msg[6]=(unsigned char)(((values[2]<<6)&0x00C0)|((values[3]>>8) &0x003F));
    msg[7]=(unsigned char)(  values[3]    &0x00FF);

    // build a can frame addressed to the group ID 
    frame = leoCAN::CANBusFrame( Group::CANID( GetID() ), msg, 8 );

    return Group::ESUCCESS;
  }
  else{
    std::cerr << LogPrefix() 
      << "Group cannot pack torques" << std::endl;
    return Group::EFAILURE;
  }

}

Group::Errno Group::GetStatus( std::vector<Barrett::Value>& status ){

  if( GetProperty( Barrett::STATUS, status ) != Group::ESUCCESS ){
    std::cerr << LogPrefix() << "Failed to query the status" 
      << std::endl;
    return Group::EFAILURE;
  }

  return Group::ESUCCESS;

}

Group::Errno Group::Initialize(){

  for( size_t i=0; i<pucks.size(); i++ ){

    std::clog << LogPrefix() << "Initalizing puck " 
      << (int)pucks[i].GetID()
      << std::endl;
    if( pucks[i].InitializeMotor() != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to initialize puck"
        << std::endl;
      return Group::EFAILURE;      
    }

  }

  return Group::ESUCCESS;

}

Group::Errno Group::SetMode( Barrett::Value mode ){

  for( size_t i=0; i<pucks.size(); i++ ){
    if( pucks[i].SetMode( mode ) != Puck::ESUCCESS ){
      std::cerr << LogPrefix() << "Failed to set the mode."
        << std::endl;
      return Group::EFAILURE;
    }
  }

  return Group::ESUCCESS;

}


