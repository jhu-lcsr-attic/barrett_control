#!/usr/bin/env python
import roslib; roslib.load_manifest('bard_components')
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def spewer():
    pub = rospy.Publisher('wam_rtt/left_joint_traj/trajectories', JointTrajectory)
    rospy.init_node('traj_spewer')
    while not rospy.is_shutdown():
        msg = JointTrajectory()

        point = JointTrajectoryPoint()
        point.positions = [0,0,0,0,0,0,0];
        point.velocities = [0,0,0,0,0,0,0];
        point.time_from_start = rospy.Duration(0.1)

        msg.joint_names = "name: ['LeftWAM/YawJoint',
        'LeftWAM/ShoulderPitchJoint', 'LeftWAM/ShoulderYawJoint',
        'LeftWAM/ElbowJoint', 'LeftWAM/UpperWristYawJoint',
        'LeftWAM/UpperWristPitchJoint', 'LeftWAM/LowerWristYawJoint']

        msg.points = [point]

        pub.publish(msg)
        print(msg)
        rospy.sleep(10.0)

if __name__ == '__main__':
    try:
        spewer()
    except rospy.ROSInterruptException: pass

