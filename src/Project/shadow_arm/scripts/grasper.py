#!/usr/bin/env python
from os import curdir
import rospy
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_msgs.msg import ContactsState
import copy

def callbacktest(msg, args):
    index = args
    #rospy.loginfo(index)
    if len(msg.states) > 0:
        flags[index] = True
    else:
        flags[index] = False

def callback2(msg):
    actual_angles = msg.actual.positions
    #rospy.loginfo(actual_angles) 
    return

def listener():

    rospy.init_node('listener', anonymous=True)
    pub_joints = rospy.Publisher('/rh_trajectory_controller/command', JointTrajectory, queue_size=10)
    for i in range(14):
        rospy.Subscriber(subscriber_topics[i], ContactsState, callbacktest, (i))
    
    increments = [0.0 for _ in range(24)]
    rospy.Subscriber("/rh_trajectory_controller/state", JointTrajectoryControllerState, callback2)
    length = 24
    rate = rospy.Rate(100)
    rospy.sleep(2.)
    next_angles = copy.deepcopy(actual_angles)
    while not rospy.is_shutdown():
        for j in range(14):
            flag = flags[j]
            k = dict[j]
            if flag:
                increments[k] = 0.0
            else:
                increments[k] = 0.1
            if j >= 12:
                increments[k] = 0.0

        next_angles = [a + b for a, b in zip(next_angles, increments)]
        first_finger = JointTrajectory()
        first_finger.joint_names = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", 
                                    "rh_LFJ1","rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
                                    "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
                                    "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
                                    "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5", 
                                    "rh_WRJ1", "rh_WRJ2"]
        newPoint = JointTrajectoryPoint()
        newPoint.time_from_start.nsecs = 1000000
        newPoint.velocities = [0.0 for _ in range(length)]
        newPoint.positions = next_angles
        first_finger.points.append(newPoint)
        pub_joints.publish(first_finger)
        rate.sleep()
            
    rospy.spin()

flags = [False for _ in range(14)]
subscriber_topics = [
"/contacts/rh_ff/distal",
"/contacts/rh_ff/middle",
"/contacts/rh_ff/proximal",
"/contacts/rh_lf/distal",
"/contacts/rh_lf/middle",
"/contacts/rh_lf/proximal",
"/contacts/rh_mf/distal",
"/contacts/rh_mf/middle",
"/contacts/rh_mf/proximal",
"/contacts/rh_rf/distal",
"/contacts/rh_rf/middle",
"/contacts/rh_rf/proximal",
"/contacts/rh_th/distal",
"/contacts/rh_th/middle",
"/contacts/rh_th/proximal"]

dict = {0: 0, 1: 1, 2:2, 3:4, 4:5, 5:6, 6:9, 7:10, 8:11, 9:13, 10:14, 11:15, 12:17, 13:18, 14:19}
actual_angles = [0.0 for _ in range(24)]

if __name__ == '__main__':
    listener()