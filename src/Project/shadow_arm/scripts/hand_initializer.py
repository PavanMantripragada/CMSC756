#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
from std_msgs.msg import String

def callback(data):
    if ( data.data == "Position the hand"):
        talker()
        sub.unregister()
    
def main():
    rospy.init_node('hand_initializer', anonymous=True)
    rospy.spin()

def talker():
    length = 24
    first_finger = JointTrajectory()
    first_finger.joint_names = ["rh_FFJ1", "rh_FFJ2", "rh_FFJ3", "rh_FFJ4", 
  "rh_LFJ1","rh_LFJ2", "rh_LFJ3", "rh_LFJ4", "rh_LFJ5",
  "rh_MFJ1", "rh_MFJ2", "rh_MFJ3", "rh_MFJ4",
  "rh_RFJ1", "rh_RFJ2", "rh_RFJ3", "rh_RFJ4",
  "rh_THJ1", "rh_THJ2", "rh_THJ3", "rh_THJ4", "rh_THJ5", 
  "rh_WRJ1", "rh_WRJ2"]
    newPoint = JointTrajectoryPoint()
    newPoint.time_from_start.secs = 1
    newPoint.velocities = [0.0 for _ in range(length)]
    newPoint.positions = [0.0 for _ in range(length)]
    first_finger.points.append(newPoint)

    newPoint2 = JointTrajectoryPoint()
    newPoint2.time_from_start.secs = 2
    newPoint2.velocities = [0.0 for _ in range(length)]
    # newPoint2.positions = [0.0, 0.0, 1.5708, 0.0, 
    # 0.0, 0.0, 1.5708, 0.0, 0.0, 
    # 0.0, 0.0, 1.5708, 0.0, 
    # 0.0, 0.0, 1.5708, 0.0, 
    # 0.0, 0.0, 0.0, 0.0, 0.0, 
    # 0.0, 0.0]

    # newPoint2.positions = [0.0013002309803011869, -0.0005731183652812888, 1.5707963267948966, -0.0040847566611059705, -0.002831573192481507, 0.0039904283167455645, 1.5707963267948966, -0.005421899417669351, -0.0003065840215104032, -8.145947540683096e-05, 0.0006961220194154905, 1.5707963267948966, -0.003671902003530292, 0.00044755422390529986, 0.0001075421982310587, 1.5707963267948966, -0.003954332143575989, 0.3665191429188092, -0.6981317007977318, -0.20943951023931956, 1.2217304763960306, 0.0, 0.00011653269466460614, -0.017453292519943295]
    newPoint2.positions = [9.45498455608984e-05, 0.0005470329130004004, 1.5707963267948966, -0.004333492448921561, 0.0018185916964217341, -0.0024123054586757675, 1.5707963267948966, -0.005411844945715671, -0.000341055516497768, 0.00010813906720930078, 0.0005048332634949304, 1.5707963267948966, -0.004082338700302834, 2.5181236004812035e-05, 0.00046193174273412296, 1.5707963267948966, -0.004251439263329715, 0.13962634015954636, 0.22689280275926285, -0.20943951023931956, 1.2217304763960306, -1.0471975511965976, 0.0001568080889375878, -0.03490658503988659]

    first_finger.points.append(newPoint2)
    
    counter = 0
    rate = rospy.Rate(100) # 10hz
    while counter < 1:
        pub.publish(first_finger)
        rate.sleep()
        counter += 1

    counter = 0
    pub2 = rospy.Publisher('/relay', String, queue_size=10)
    while counter < 1000:
        pub2.publish("Move Hand")
        rate.sleep()
        counter += 1
    return
    
pub = rospy.Publisher('/rh_trajectory_controller/command', JointTrajectory, queue_size=10)
sub = rospy.Subscriber("/relay", String, callback)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass