import numpy as np
import matplotlib.pyplot as plt

class Polygon:
    def __init__(self,vertex_list,convexity):
        self.vertex_list = vertex_list
        self.convexity = convexity
        if convexity:
            self.create_convex_polygon()
        else:
            self.create_nonconvex_polygon()
    def create_convex_polygon(self):
        self.edge_list = []
        for idx in range(len(self.vertex_list)-1):
            self.edge_list.append([self.vertex_list[idx], self.vertex_list[idx+1]])
        self.edge_list.append([self.vertex_list[-1], self.vertex_list[0]])
    def create_nonconvex_polygon(self):
        self.edge_list = []
        for idx in range(len(self.vertex_list)-1):
            self.edge_list.append([self.vertex_list[idx], self.vertex_list[idx+1]])
        self.edge_list.append([self.vertex_list[-1], self.vertex_list[0]])
        self.child_polygon_list = []
    def contains(self,x,y):
        if self.convexity:
            count1 = 0
            count2 = 0
            for [x1,y1],[x2,y2] in self.edge_list:
                s = (y-y2)/(y1-y2)
                x_intercept = s*x1+(1-s)*x2 
                if s > 0.0:
                    if s<=1.0:
                        if x_intercept > x:
                            count1 +=1
                        elif x_intercept < x:
                            count2 +=1
                        else:
                            return True    
            if count1 % 2 != 0 and count2 % 2 != 0:
                return True
            return False
        else:
            for child_polygon in self.child_polygon_list:
                count1 = 0
                count2 = 0
                for [x1,y1],[x2,y2] in child_polygon.edge_list:
                    s = (y-y2)/(y1-y2)
                    x_intercept = s*x1+(1-s)*x2 
                    if s > 0.0:
                        if s<=1.0:
                            if x_intercept > x:
                                count1 +=1
                            elif x_intercept < x:
                                count2 +=1
                            else:
                                return True    
                if count1 % 2 != 0 and count2 % 2 != 0:
                    return True
            return False
    def intersects(self,q1,q2):
        for p1,p2 in self.edge_list:
            status,t = line_segement_collision(p1,p2,q1,q2)
            if status:
                qn = [[(1-t)*q1[0]+t*q2[0]],[(1-t)*q1[1]+t*q2[1]]]
                return True, qn
        return False, None

class Obstacles:
    def __init__(self):
        self.obstacle_list = []
    def add_obstacle(self,polygon):
        self.obstacle_list.append(polygon)

class Boundaries:
    def __init__(self):
        self.boundary_list = [[[0,0],[30,0]],[[30,0],[30,17]],
                              [[30,17],[0,17]],[[0,17],[0,0]]]
        self.lim_xl = 0
        self.lim_xu = 30
        self.lim_yl = 0
        self.lim_yu = 17 
    def contains(self,x,y):
        if x>self.lim_xl and x<self.lim_xu \
             and y>self.lim_yl and y<self.lim_yu:
            return True
        else:
            return False
    def intersects(self,q1,q2):
        for p1,p2 in self.boundary_list:
            status,t = line_segement_collision(p1,p2,q1,q2)
            if status:
                qn = [[(1-t)*q1[0]+t*q2[0]],[(1-t)*q1[1]+t*q2[1]]]
                return True, qn
        return False, None

class Node:
    def __init__(self,x,y,parent_node=None):
        self.x = x
        self.y = y
        self.parent = parent_node     
    def __eq__(self,other):
        if self.x == other.x  and self.y == other.y:
            return 1
        else:
            return 0

class Point_robot:
    def __init__(self,obstacles,boundaries):
        self.obstacles = obstacles
        self.boundaries = boundaries
        self.start = [1,1]
        self.goal = [29,16]
        self.fig, self.ax = plt.subplots()
        self._init_map()

    def _init_map(self):
        self.ax.set_title("Map")
        self.ax.set_xlabel("X axis")
        self.ax.set_ylabel("Y axis")
        for obstacle in self.obstacles.obstacle_list:
            for [x1,y1],[x2,y2] in obstacle.edge_list:
                self.ax.plot([x1,x2],[y1,y2],c='k')
        for [x1,y1],[x2,y2] in self.boundaries.boundary_list:
            self.ax.plot([x1,x2],[y1,y2],c='k')            
        self.ax.set_xlim(-1,31)
        self.ax.set_ylim(-1,18)
        plt.pause(0.01)

    def is_collision_free_between(self,q1,q2):
        for obstacle in self.obstacles.obstacle_list:
            status, qn = obstacle.intersects(q1,q2)
            if status:
                return False, qn
        for boundary in self.boundaries.boundary_list:
            status, qn = boundary.intersects(q1,q2)
            if status:
                return False, qn
        return True, q2

    def find_nearest_node(self,qn):
        minDistance = float("inf")
        q_near = None
        for q in self.visited_list:
            currentDistance = np.linalg.norm([q[0]-qn[0],q[1]-qn[1]])
            if minDistance > currentDistance:
                minDistance = currentDistance
                q_near = [q[0],q[1]]
        return q_near

    def generate_RRT(self,start,goal):
        self.visited_list = {}
        start_node = Node(start[0],start[1])
        self.visited_list[(start[0],start[1])] = start_node
        distance_to_goal = float("inf")
        while distance_to_goal < 1.0:
            q_new = np.random.rand(2)*
            if tuple(q_new) in self.visited_list:
                continue
            status = False
            q_near = self.find_nearest_node(q_new)
            while not status:
                status, q_new = self.is_collision_free_between(q_near,q_new)
            new_node = Node(q_new[0],q_new[1],self.visited_list[tuple(q_near)])
            self.visited_list[tuple(q_new)] = new_node
            distance_to_goal = np.linalg.norm([goal[0]-q_new[0],goal[1]-q_new[1]])
        return new_node

    def backtrack(self,final_node):
        path = []
        node = final_node
        while node != None:
            path.append([node.x,node.y])
            node = node.parent
        path.reverse()
        return path
    
    def plot_path(self,path):
        data = np.array(path)
        self.ax.plot(data[:,0],data[:,1],'-o',c='g')


def line_segement_collision(a,b,c,d):
    # Line segment 1 will be (a,b)
    # Line segment 2 will be (c,d)
    # Collision is checked by finding
    # atleast one common point 'p' such that 
    # p = (1-s)*a+s*b and also p = (1-t)*c+t*d
    # where 0 <= s,t <= 1

    l1,l2 = np.zeros(3),np.zeros(3)
    
    # floating point error for checking singularity
    eps = 1e-10

    # Solving for common point on the line segments
    # using Cramers rule
    D = np.linalg.det(np.array([[b[0]-a[0],c[0]-d[0]],
                                [b[1]-a[1],c[1]-d[1]]]))  
    D1 = np.linalg.det(np.array([[c[0]-a[0],c[0]-d[0]],
                                 [c[1]-a[1],c[1]-d[1]]]))
    D2 = np.linalg.det(np.array([[b[0]-a[0],c[0]-a[0]],
                                 [b[1]-a[1],c[1]-a[1]]])) 
   
    # If the a single common point exists 
    if abs(D) > eps:
        s = D1/D
        t = D2/D
        
        # Checking if the common point lies beyond both the segments
        if s >= 0.0 and s<=1.0 and t >= 0.0 and t<=1.0:
            return True, t-0.001
        else:
            return False, None
    else:
        # Check if there exists multiple common points
        if abs(D1) <= eps and abs(D2) <= eps:
            if c[0] != d[0]:
                t1 = (a[0]-c[0])/(d[0]-c[0])
                t2 = (b[0]-c[0])/(d[0]-c[0])
            else:
                t1 = (a[1]-c[1])/(d[1]-c[1])
                t2 = (b[1]-c[1])/(d[1]-c[1])
            ts = []
            if t1>=0.0 and t1<=1.0:
                ts.append(t1)
            if t2>=0.0 and t2<=1.0:
                ts.append(t2)
            # Checking if any common points lie in the segments
            if len(ts):
                return True, max(ts)-0.001    
            else:
                return False, None
        else:
            return False, None

def main():

    obstacles = Obstacles()
    boundaries = Boundaries()
    obstacle1 = Polygon([[4,3],[7,9],[9,7]],True)
    obstacle2 = Polygon([[14,3],[17,9],[19,7]],True)
    obstacles.add_obstacle(obstacle1)
    obstacles.add_obstacle(obstacle2)

    my_robot = Point_robot(obstacles,boundaries)
    end_node = my_robot.generate_RRT([1,1],[29,16])
    path = my_robot.backtrack(end_node)
    my_robot.plot_path(path)
    plt.show()

if __name__ == '__main__':
    main()