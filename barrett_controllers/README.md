

Calibration Routine
-------------------

### Saving Calibration Data

1. Start the calibration client, which will start the appropriate controllers:

    rosrun barrett_calibration position_calibration

1. Use keys 1-7 to increment the angle by one count, and keys q-u to decrement
   the angle by one count until you're satisfied that the robot is in the
   desired calibration position, then press Enter.
1. Finally, type in the angles of each joint (in degrees) to generate the
   calibration parameter string, which will be printed to the screen. This
   array of angles are the motor angles in joint space.

### Usng Calibration Data

1. Start the WAM hanging in a neutral position
1. Calibration of each joint occurs from the tip (wrist yaw) back to the root
   (base yaw)
  1. Apply small torque to drive to the semi-hard stop
  1. Detect semi-hard stop when _uncalibrated_ position stops changing
  1. At this stop, we know the _actual_ position within a few degrees
  1. Apply joint-level PID to get within one motor revolution of a joint
     position near which we have calibrated the actual motor angle
  1. Once at this joint position, compute the joint angle offset from the
     difference between the calibrated motor angle and the actual motor angle 
  1. Add the joint angle offset to the known joint position, and declare that
     to be the actual position
  1. Apply joint-level PID to reach safe position for next joint
  1. Continue to next joint

