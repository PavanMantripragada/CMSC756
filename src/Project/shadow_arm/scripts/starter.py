#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('relay', String, queue_size=10)
    rospy.init_node('starter', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    hello_str = "Position the hand"
    counter = 0
    while counter < 10:
        pub.publish(hello_str)
        rate.sleep()
        counter += 1
    return
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass