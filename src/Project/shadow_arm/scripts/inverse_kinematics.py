import numpy as np
import matplotlib.pyplot as plt

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
            T = T @ self.A(i)
        o = T[0:3,3]
        return o
    
    def z(self,n):
        T = self.T_world
        for i in range(n):
            T = T @ self.A(i)
        z = T[0:3,2]
        return z

    def T(self,n):
        T = self.T_world
        for i in range(n):
            T = T @ self.A(i)
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
        Xdot = self.J() @ qdot.reshape(6,1)
        return Xdot

    def IVK_circle(self,t):
        omega = 2*np.pi/60
        radius = 0.766 # mm
        xdot = -radius*omega*np.sin(omega*t + np.pi/2 + np.pi/12)
        zdot = 0
        ydot = radius*omega*np.cos(omega*t + np.pi/2 + np.pi/12)
        # Assuming that the tool frame doesn't rotate
        pitch_velocity =  omega
        Xdot = np.array([xdot,ydot,zdot,0,0,0]).reshape(6,1)
        Jinv = np.linalg.inv(self.J())
        qdot = Jinv @ Xdot
        qdot = qdot.reshape(6,1)
        return qdot, Jinv

    def show(self,fig,ax):
        ax.clear()
        # Link 1
        j1 = self.o(0)
        j2 = self.o(1)
        j4 = self.o(3)
        j6 = self.o(5)
        ee = self.o(6)

        x = [j1[0],j2[0]]
        y = [j1[1],j2[1]]
        z = [j1[2],j2[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        x = [j2[0],j4[0]]
        y = [j2[1],j4[1]]
        z = [j2[2],j4[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        x = [j4[0],j6[0]]
        y = [j4[1],j6[1]]
        z = [j4[2],j6[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        x = [j6[0],ee[0]]
        y = [j6[1],ee[1]]
        z = [j6[2],ee[2]]
        ax.plot(x,y,z,color='k',linewidth=5)
        A = self.A(0) @ self.A(1) @ self.A(2) @ self.A(3) @ self.A(4) @ self.A(5) @ self.A(6)
        o =  A @ np.array([0,0,0,1]).T
        vx = A[0:3,0:3] @ np.array([100,0,0])
        vy = A[0:3,0:3] @ np.array([0,100,0])
        vz = A[0:3,0:3] @ np.array([0,0,100])
        ax.quiver(o[0],o[1],o[2],vx[0],vx[1],vx[2],color='r')
        ax.quiver(o[0],o[1],o[2],vy[0],vy[1],vy[2],color='g')
        ax.quiver(o[0],o[1],o[2],vz[0],vz[1],vz[2],color='b')
        ax.set_xlim3d(-800,800)
        ax.set_ylim3d(-600,1000)
        ax.set_zlim3d(0,1000)
        fig.canvas.draw()
        fig.canvas.flush_events()

def print_list(a,text):
    print("List of " + text + " :")
    for i in a:
        print(i)
        print("---------")
    print("################################")

def main():
     
    d = np.array([0.1273,0,0,0.163941,0.1157,0.0922])    
    q = np.array([0.09523985696467108, -0.7183862276793844, 0.48845086571663465, 0.22925462512416583, 1.6656604751690187, -1.571821596664277])
    q = q.reshape(6,1)
    robot = UR10(q,d)
    robot.T_world = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0.765],[0,0,0,1]]) 

    # print("Transformation matrices from each frame to ground frame :")
    # for i in range(1,7):
    #     print("Tranformation matrix of frame",i)
    #     print(robot.T(i))
    #     print("---------")
    # print("################################")
    # print(robot.o(6))
    end_time = 10
    time_steps = 2000
    dt = end_time/time_steps
    T = np.linspace(0,end_time,time_steps)
    
    fig = plt.figure()
    ax1 = fig.add_subplot(projection='3d')
    plt.ion()
    plt.axis('auto')
    
    # Simulating the robot over time
    print("Simulation has started please wait for the output")
    print("Note: The axes of the output are not equally scaled!!!")
    print("So circles might appear as ellipses")
    Jinv_list = []
    qdot_list = []
    o_list = []
    for t in T:
        qdot,Jinv = robot.IVK_circle(t)
        q = robot.get_joints()
        q += qdot*dt
        robot.set_joints(q)
        o = robot.o(6).tolist()
        ax1.plot(o[0],o[1],o[2],c='k',marker='.')
        Jinv_list.append(Jinv)
        qdot_list.append(qdot)
        o_list.append(o)    
    # ax1.set_xlim(-1,1)
    # ax1.set_ylim(-2,2)
    ax1.set_zlim(0,2)
    ax1.set_xlabel("X axis")
    ax1.set_ylabel("Y axis")
    ax1.set_zlabel("Z axis")
    ax1.set_title("Trajectory of end effector")
    plt.pause(340)


if __name__ == '__main__':
    main()


