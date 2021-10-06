import numpy as np
import matplotlib.pyplot as plt
import argparse

class elbow_manipulator:
    
    def __init__(self,d1,d2,a2,d4,d6):
        # DH parameters of the bot
        self.a = [0.0,a2,0.0,0.0,0.0,0.0]
        self.alpha = [-np.pi/2,0.0,np.pi/2,-np.pi/2,np.pi/2,0.0]
        self.d = [d1,d2,0.0,d4,0.0,d6]
        self.q = [0.0,0.0,0.0,0.0,0.0,0.0]

    def reach(self,o,R):
        a = self.a
        d = self.d

        # Computing wrist center from tool frame pose
        x = o[0] - d[5]*R[0][2]
        y = o[1] - d[5]*R[1][2]
        z = o[2] - d[5]*R[2][2]

        # Checking for regions beyond the reach of arm
        # closer to the z0 axis
        if x**2+y**2 < d[1]**2:
            print("Sorry the following pose is outside the workspace!")
            return 0
        
        # Checking for singularity when shoulder offset is zero
        if x <= 1e-15 and y == 1e-15:
            print("Singular Position: Generating a solution with joint angle 1 to be zero")
            self.q[0] = 0
        else:
            # Always choosing the left arm configuration
            q1 = -np.pi/2 + np.arctan2(y,x) + np.arctan2(np.sqrt(x**2+y**2-d[1]**2),d[1])

            # Ensuring q1 lies between [-pi,pi]
            if q1 > np.pi:
                q1 = q1 - 2*np.pi
            self.q[0] = q1

        D = (x**2+y**2-d[1]**2
             +(z-d[0])**2-a[1]**2-d[3]**2)/(2*a[1]*d[3])
        # Checking for regions beyond the reach of arm
        if np.abs(D) > 1.0:
            print("Sorry the following pose is outside the workspace!")
            return 0
        
        # Choosing one elbow configuration
        self.q[1] = -np.arctan2(z-d[0],np.sqrt(x**2+y**2-d[1]**2)) + np.arctan2(
                    d[3]*np.sqrt(1-D**2),a[1]+d[3]*D)
        self.q[2] = -np.arctan2(np.sqrt(1-D**2),D) + np.pi/2
        
        # Computing Spherical Wrist angles as Euler Angles
        R_30 = (self.A(0) @ self.A(1) @ self.A(2))[0:3,0:3]
        R_63 = R_30.T @ R
 
        if abs(R_63[0][2]) <= 1e-15 and abs(R_63[1][2]) <= 1e-15:
            if R_63[2][2] > 0.0:
                self.q[3] = 0.0
                self.q[4] = 0.0
                self.q[5] = np.arctan2(R_63[1][0],R_63[0][0])
            if R_63[2][2] < 0.0:
                self.q[3] = np.arctan2(-R_63[0][1],-R_63[0][0])
                self.q[4] = np.pi
                self.q[5] = 0.0
        else:
            self.q[3] = np.arctan2(R_63[1][2],R_63[0][2])
            self.q[4]= np.arctan2(np.sqrt(1-R_63[2][2]**2),R_63[2][2])
            self.q[5] = np.arctan2(R_63[2][1],-R_63[2][0])

        return 1

    def display_configuration(self):
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        fig.set_size_inches(6, 6)
        plt.axis('auto')

        # Link 1
        p0 = np.array([0,0,0,1])
        p1 = self.A(0) @ p0.T
        x = [p0[0],p1[0]]
        y = [p0[1],p1[1]]
        z = [p0[2],p1[2]]
        ax.plot(x,y,z,color='r',linewidth=5)
        p2 = self.A(0) @ np.array([0,0,self.d[1],1]).T
        x = [p1[0],p2[0]]
        y = [p1[1],p2[1]]
        z = [p1[2],p2[2]]        
        ax.plot(x,y,z,color='r',linewidth=5)

        # Link 2
        p3 = self.A(0) @ self.A(1) @ np.array([0,0,0,1]).T
        x = [p2[0],p3[0]]
        y = [p2[1],p3[1]]
        z = [p2[2],p3[2]]        
        ax.plot(x,y,z,color='g',linewidth=5)

        # Link 3
        p4 = self.A(0) @ self.A(1) @ self.A(2) @ self.A(3) @ np.array([0,0,0,1]).T
        x = [p3[0],p4[0]]
        y = [p3[1],p4[1]]
        z = [p3[2],p4[2]]        
        ax.plot(x,y,z,color='b',linewidth=5)
        
        # tool frame
        A = self.A(0) @ self.A(1) @ self.A(2) @ self.A(3) @ self.A(4) @ self.A(5)
        print("Tool-frame according to computed joint angles:\n", np.round(A[0:3,0:3],2))
        o =  A @ np.array([0,0,0,1]).T
        x = [p4[0],o[0]]
        y = [p4[1],o[1]]
        z = [p4[2],o[2]]        
        ax.plot(x,y,z,color='k',linewidth=5)
        vx = A[0:3,0:3] @ np.array([0.5,0,0])
        vy = A[0:3,0:3] @ np.array([0,0.5,0])
        vz = A[0:3,0:3] @ np.array([0,0,0.5])
        ax.quiver(o[0],o[1],o[2],vx[0],vx[1],vx[2],color='r')
        ax.quiver(o[0],o[1],o[2],vy[0],vy[1],vy[2],color='g')
        ax.quiver(o[0],o[1],o[2],vz[0],vz[1],vz[2],color='b')
        
        h_lim = self.d[1] + abs(self.a[1]) + self.d[3] + self.d[5]
        v_lim1 = self.d[0] -abs(self.a[1]) - self.d[3] - self.d[5]
        v_lim2 = abs(self.a[1]) + self.d[3] + self.d[5] + self.d[0]
        ax.set_xlim3d(-h_lim,h_lim)
        ax.set_ylim3d(-h_lim,h_lim)
        ax.set_zlim3d(v_lim1,v_lim2)
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")
        ax.set_zlabel("Z axis")
        print("Tool-frame origin according to computed joint angles:\n",o[0:3])
        plt.pause(60)
    
    def A(self,i):
        a = self.a
        alpha = self.alpha
        d = self.d
        q = self.q
        A = np.array([[np.cos(q[i]), -np.sin(q[i])*np.cos(alpha[i]),
                       np.sin(q[i])*np.sin(alpha[i]), a[i]*np.cos(q[i])],
                      [np.sin(q[i]), np.cos(q[i])*np.cos(alpha[i]),
                       -np.cos(q[i])*np.sin(alpha[i]), a[i]*np.sin(q[i])],
                      [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                      [0, 0, 0, 1]])
        return A   


def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--x",
        default=1,
        help="x coordinate of tool frame origin")
    parser.add_argument(
        "--y",
        default=-1,
        help="y coordinate of tool frame origin")
    parser.add_argument(
        "--z",
        default=1,
        help="z coordinate of tool frame origin")
    args = parser.parse_args()    

    # robot link dimensions
    d1 = 2
    d2 = 1    # shoulder offset
    a2 = 1.5    
    d4 = 1    # it is equivalent to a3
    d6 = 0.5
    robot = elbow_manipulator(d1,d2,a2,d4,d6)
    
    # Processing inputs
    o = np.array([float(args.x),float(args.y),float(args.z)])
    R = np.identity(3)
    print("Input tool frame origin:\n",o)
    print("Input tool frame orientation:\n",R)
    
    # Checking for reachability
    if robot.reach(o,R):
        print("Obtained Joint Angles:\n",np.array(robot.q)*180/np.pi)
        robot.display_configuration()

if __name__ == '__main__':
    main()