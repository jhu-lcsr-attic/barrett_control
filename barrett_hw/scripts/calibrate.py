#!/usr/bin/env python

import roslib; roslib.load_manifest('barrett_hw')
import rospy


def main():
    rospy.init_node('wam_calibration')

    # Create service server / actionlib server
    # Check calibration parameter
    # Load and start calibration controller

    rosservice call /wam/calibration_controller/calibrate "joint_names: ['wam/UpperWristPitchJoint','wam/UpperWristYawJoint','wam/LowerWristYawJoint','wam/ShoulderYawJoint']"
    rosservice call /wam/calibration_controller/calibrate "joint_names: ['wam/ElbowJoint']"
    rosservice call /wam/calibration_controller/calibrate "joint_names: ['wam/ShoulderPitchJoint']"
    rosservice call /wam/calibration_controller/calibrate "joint_names: ['wam/YawJoint']"

    # Set calibrated rosparam

    # Load gravity compensation controller

    # Switch  calibration controller with gravity compensation controller

    # Unload calibration controller
    
if __name__ == '__main__':
    main()
