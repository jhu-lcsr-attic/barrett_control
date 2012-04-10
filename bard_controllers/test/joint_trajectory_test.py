#!/usr/bin/env python
import roslib; roslib.load_manifest('bard_controllers')
import rospy
import math
from copy import copy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def spewer():
    pub = rospy.Publisher('wam_rtt/left_joint_traj/trajectories', JointTrajectory)
    rospy.init_node('traj_spewer')
    while not rospy.is_shutdown():
        msg = JointTrajectory()

        msg.header.stamp = rospy.Time.now()

        msg.joint_names = ['LeftWAM/YawJoint',
                'LeftWAM/ShoulderPitchJoint', 'LeftWAM/ShoulderYawJoint',
                'LeftWAM/ElbowJoint', 'LeftWAM/UpperWristYawJoint',
                'LeftWAM/UpperWristPitchJoint', 'LeftWAM/LowerWristYawJoint']

        msg.points=[]

        point = JointTrajectoryPoint()
        point.velocities = [0,0,0,0,0,0,0]

        point.positions = [0,0,0,math.pi,0,0,0]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(copy(point))

        point.positions = [0,-math.pi/2.0,0,math.pi,0,0,0]
        point.time_from_start = rospy.Duration(8.0)
        msg.points.append(copy(point))

        '''
        point.time_from_start = rospy.Duration(2.0)
        msg.points.append(copy(point))
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(copy(point))
        point.time_from_start = rospy.Duration(4.0)
        msg.points.append(copy(point))
        point.time_from_start = rospy.Duration(5.0)
        msg.points.append(copy(point))
        '''


        pub.publish(msg)
        print(msg)
        rospy.sleep(15.0)

if __name__ == '__main__':
    try:
        spewer()
    except rospy.ROSInterruptException: pass

