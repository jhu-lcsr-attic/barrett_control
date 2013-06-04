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

#ifndef __BARRETT_DIRECT_GROUP_H
#define __BARRETT_DIRECT_GROUP_H

#include <vector>

#include <Eigen/Dense>

#include <leo_can/CANBus.h>

#include <barrett_direct/Puck.h>

//! A logical group of pucks
/**
  Groups are used to communicate with pucks simultaneously. This has the 
  benifit of saving bandwidth. For example, all the pucks belong to the 
  BROADCAST group. Thus, when a CAN frame is destined to the BROADCAST group
  all the pucks will process the frame.
  */
namespace barrett_direct {

  class Group {
  public:

    //! The ID used to identify each group
    enum ID{

      //! The broadcast group
      /**
        The broadcast group contains all the pucks in a WAM (with the exception 
        of the safety module)
        */
      BROADCAST = 0x00,

      //! The upper arm group
      /**
        The upper arm group represents the 4 pucks of the upper arm 
        (shoulder+elbow)
        */
      UPPERARM = 0x01,

      //! The forearm group
      /**
        The forearm group represents the 3 pucks of the upper arm (wrist)
        */
      FOREARM = 0x02,

      //! The motor position group
      /**
        All pucks belong to the position group. Send a message to this group to 
        query all the motor positions. Each puck will reply to a message to this
        group with its motor position
        */
      POSITION = 0x03,

      //! Upper arm property group
      UPPERARM_POSITION = 0x04,

      //! Forearm property group
      FOREARM_POSITION = 0x05,

      //! Feedback property group
      PROPERTY = 0x06,

      HAND = 0x07,

      HAND_POSITION = 0x08,

      LASTGROUP = 0x09

    };

    //! Define the status of a group
    /**
      Barrett defines the following mode in which a puck can be
      */
    enum Status{ RESET=0, READY=2 };

    //! Error codes used by Group
    enum Errno{ ESUCCESS, EFAILURE };

  private:

    std::string LogPrefix();

    //! The ID of the pucks in the group
    std::vector< Puck > pucks;

    //! The CAN bus that is connected to the group
    leo_can::CANBus* canbus; 

    //! The ID of the group
    Group::ID id;

    //! Is the data contain a set property command
    /**
      Pucks have properties that can be read/write. To read/write a property, 
      you must send a CAN frame with the proper data format. To write a property,
      the CAN data must have a "write" bit set while a read command must have the
      "write" bit cleared. This method returns true if the "write" bit is set.
      \param canframe A CAN frame with a read/write command
      \return true if the command is a write. false if the command is a read
      */
    static bool IsSetFrame( const leo_can::CANBusFrame& canframe );

    //! pack a CAN frame
    /**
      Build a CAN frame destined to the group of pucks with the data formated.
      \param canframe[out] The resulting CAN frame
      \param propid The property ID to set or get
      \param propval The property value if the property must be set
      \param set True of the property must be set. False for a query
      \return false if no error occurred. true otherwise
      */
    Group::Errno PackProperty( leo_can::CANBusFrame& canframe,
        Barrett::Command command,
        Barrett::ID propid,
        Barrett::Value propval = 0 );

    //! The bit of a CAN ID that identicates a group
    static const leo_can::CANBusFrame::id_t GROUP_CODE = 0x0400;


    Group::Errno PackCurrents( leo_can::CANBusFrame& frame, 
        const Eigen::Vector4d& I );

    //! Querry the group. This is only valid for querying
    //  positions on group 3
    Group::Errno GetProperty( Barrett::ID id, 
        std::vector<Barrett::Value>& values );

    //! Set the property of a group
    Group::Errno SetProperty( Barrett::ID id, 
        Barrett::Value value,
        bool verify );

  public:

    //! Create a group with an ID and a CAN device
    /**
      Initialize the group to the given ID and give the CAN device connected to
      the pucks.
      \param groupid The ID of the puck
      \param can The CAN device used to communicate with the pucks
      */
    Group( Group::ID id, leo_can::CANBus* canbus, bool createfilter=true );

    //! Convert a group ID to a CAN id
    /**
      Convert the ID of a group to a CAN ID used in a CAN frame. This assumes 
      that the origin of the CAN ID will be the host (00000)
      */
    static leo_can::CANBusFrame::id_t CANID( Group::ID groupid );

    //! Return the group ID
    Group::ID GetID() const;

    //! Return the origin ID of the CAN id
    static Group::ID OriginID( leo_can::CANBusFrame::id_t canid );

    //! Return the origin ID of the CAN frame
    static Group::ID OriginID( const leo_can::CANBusFrame& canframe );

    //! Return the destination ID of the CAN id
    static Group::ID DestinationID( leo_can::CANBusFrame::id_t canid );

    //! Return the destination ID of the CAN id
    static Group::ID DestinationID( const leo_can::CANBusFrame& canframe );

    //! Return true if the CAN frame id's destination is a group (any group)
    static bool IsDestinationAGroup( const leo_can::CANBusFrame canframe );

    //! Add the puck ID to the group
    void AddPuckToGroup( Puck::ID pid );

    bool Clear() const { return pucks.empty(); }

    //! Return the puck ID of the first member
    Puck First() const { return pucks.front(); }
    Puck Last()  const { return pucks.back(); }

    bool IsEmpty() const { return pucks.empty(); }


    Group::Errno Initialize();

    Group::Errno Reset();

    Group::Errno Ready();

    Group::Errno GetStatus( std::vector<Barrett::Value>& status );


    Group::Errno GetPositions( Eigen::VectorXd& q );
    Group::Errno SetTorques( const Eigen::Vector4d& tau );

    Group::Errno SetMode( Barrett::Value mode );



  };

  // Increment operator for pucks id
  Group::ID operator++( Group::ID& gid, int i );
}

#endif // ifndef __BARRETT_DIRECT_GROUP_H
