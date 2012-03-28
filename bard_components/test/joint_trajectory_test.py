#!/usr/bin/env python
import roslib; roslib.load_manifest('bard_components')
import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def spewer():
    pub = rospy.Publisher('wam_rtt/left_joint_traj/trajectories', JointTrajectory)
    rospy.init_node('traj_spewer')
    while not rospy.is_shutdown():
        msg = JointTrajectory()

        #msg.header.stamp = rospy.Time.now()

        point = JointTrajectoryPoint()
        point.positions = [0,3.1415,0,0,0,0,0]
        point.velocities = [0,0,0,0,0,0,0]
        point.time_from_start = rospy.Duration(5.0)

        msg.joint_names = ['LeftWAM/YawJoint',
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

