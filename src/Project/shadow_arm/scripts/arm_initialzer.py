#!/usr/bin/env python3
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

def callback(data):
    if ( data.data == "Open Door"):
        talker()
        sub.unregister()

class UR10:
    def __init__(self,q,d):
        self.q = q
        self.d = d
        self.alpha = np.array([90,0,0,90,-90,0])*(np.pi/180)
        self.a = np.array([0,-0.612,-0.5723,0,0,0])
        self.T_world = np.identity(4)

    def get_joints(self):
        return self.q

    def set_joints(self,q):
        self.q = q.reshape(6,1)
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q.reshape(6,)
        A = np.array([[np.cos(q[i]), -np.sin(q[i])*np.cos(alpha[i]),
                       np.sin(q[i])*np.sin(alpha[i]), a[i]*np.cos(q[i])],
                      [np.sin(q[i]), np.cos(q[i])*np.cos(alpha[i]),
                       -np.cos(q[i])*np.sin(alpha[i]), a[i]*np.sin(q[i])],
                      [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        return A   

    def o(self,n):
        T = self.T_world
        for i in range(n):
            T = np.matmul(T, self.A(i)) 
        o = T[0:3,3]
        return o
    
    def z(self,n):
        T = self.T_world
        for i in range(n):
            T = np.matmul(T, self.A(i)) 
        z = T[0:3,2]
        return z

    def T(self,n):
        T = self.T_world
        for i in range(n):
            T = np.matmul(T, self.A(i)) 
        return T
    
    def J(self):
        j = np.zeros((6,1))
        for i in range(1,7):
            p = np.cross(self.z(i-1),self.o(6)-self.o(i-1)).reshape(3,1)
            p = np.vstack((p,self.z(i-1).reshape(3,1)))
            j = np.hstack((j,p))
        jacobian = j[:,1:]
        if abs(np.linalg.det(jacobian)) <= 1e-1:
            #print("singular warning!", abs(np.linalg.det(jacobian)))
            pass

        return jacobian

    def FVK(self,qdot):
        Xdot =  np.matmul(self.J(), qdot.reshape(6,1)) 
        return Xdot

    def IVK_circle(self,t):
        omega = 2*np.pi/60
        radius = 0.766 # mm
        r2 = 0.720
        xdot = -radius*omega*np.sin(omega*t + np.pi/2 + np.pi/12)
        zdot = 0
        ydot = radius*omega*np.cos(omega*t + np.pi/2 + np.pi/12)
        # Assuming that the tool frame doesn't rotate
        pitch_velocity =  omega
        Xdot = np.array([xdot,ydot,zdot,0,0,0]).reshape(6,1)
        Jinv = np.linalg.inv(self.J())
        qdot = np.matmul(Jinv, Xdot )
        qdot = qdot.reshape(6,1)
        return qdot, Jinv

def talker():

    d = np.array([0.1273,0,0,0.163941,0.1157,0.0922])    
    q = np.array([0.09523985696467108, -0.7183862276793844, 0.48845086571663465, 0.22925462512416583, 1.6656604751690187, -1.571821596664277])
    q = q.reshape(6,1)
    robot = UR10(q,d)
    robot.T_world = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0.765],[0,0,0,1]]) 

    end_time = 10
    time_steps = 2000
    dt = end_time/time_steps
    T = np.linspace(0,end_time,time_steps)
    
    arm = JointTrajectory()
    arm.joint_names = ["ra_shoulder_pan_joint", "ra_shoulder_lift_joint", "ra_elbow_joint", "ra_wrist_1_joint",
  "ra_wrist_2_joint", "ra_wrist_3_joint"]

#     arm.joint_names = ["ra_elbow_joint", "ra_shoulder_lift_joint", "ra_shoulder_pan_joint", "ra_wrist_1_joint",
#   "ra_wrist_2_joint", "ra_wrist_3_joint"]

    print("computing")
    counter = 0
    time_counter = 1
    length = 6

    for t in T:
        qdot,Jinv = robot.IVK_circle(t)
        # print(qdot)
        q = robot.get_joints()
        # print(q)
        q += qdot*dt
        print(q)
        robot.set_joints(q)
        if ( counter % 100  == 0):
            newPoint = JointTrajectoryPoint()
            newPoint.velocities = [ 0.0 for _ in range(length)]
            newPoint.positions = q.reshape(6).tolist()
            # print(q * 180/np.pi)
            newPoint.time_from_start.secs = time_counter      
            arm.points.append(newPoint)
            time_counter += 1
        counter += 1
        
    print("completed")
    counter = 0
    rate = rospy.Rate(100) # 10hz
    while counter < 100:
        pub.publish(arm)
        rate.sleep()
        counter += 1

pub = rospy.Publisher('/ra_trajectory_controller/command', JointTrajectory, queue_size=10)
sub = rospy.Subscriber("/relay", String, callback)


def main():
    rospy.init_node('arm_intializer', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main()


