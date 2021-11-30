import numpy as np
import argparse
import matplotlib.pyplot as plt
import polygons as ply
import linesegments as ls

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
            collision_status,t = ls.line_segement_collision(p1,p2,q1,q2)
            if collision_status:
                return True, t
        return False, None

class Node:
    def __init__(self,x,y,parent_node=None):
        self.x = x
        self.y = y
        self.parent = parent_node     

class Point_robot:
    def __init__(self,*args):
        self.radius = 3
        self.fig, self.ax = plt.subplots()
        self.start = None
        self.goal = None
        self.boundaries = args[1]
        if len(args) > 2:
            self._init_map_obs(args[2])
        else:
            self.obstacles = args[0]
            self._init_map()    

    def _init_map_obs(self,num_obs):
        self.ax.set_title("Map")
        self.ax.set_xlabel("X axis")
        self.ax.set_ylabel("Y axis")
        for [x1,y1],[x2,y2] in self.boundaries.boundary_list:
            self.ax.plot([x1,x2],[y1,y2],c='k')            
        self.ax.set_xlim(-1,31)
        self.ax.set_ylim(-1,18)
        plt.pause(0.01)
        self.obstacles = Obstacles()
        for i in range(num_obs):
            vertices = plt.ginput(n=-1,timeout=30)
            vertex_list = [list(vertex) for vertex in vertices]
            self.obstacles.add_obstacle(ply.Polygon(vertex_list,False))

        for obstacle in self.obstacles.obstacle_list:
            for [x1,y1],[x2,y2] in obstacle.edge_list:
                self.ax.plot([x1,x2],[y1,y2],c='k')
        [(x1,y1),(x2,y2)] = plt.ginput(n=2,timeout=120)
        self.goal = [x2,y2]
        if self.is_in_free_space([x1,y1]):
            self.start = [x1,y1]
            self.ax.plot(x1,y1,c='r',marker='o',markersize=12)    
        else:
            print("start location is not in free space!")
        if self.is_in_free_space([x2,y2]):
            self.goal = [x2,y2]
            self.ax.plot(x2,y2,c='b',marker='o',markersize=12)    
        else:
            print("goal location is not in free space!")

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
        [(x1,y1),(x2,y2)] = plt.ginput(n=2,timeout=30)
        self.goal = [x2,y2]
        if self.is_in_free_space([x1,y1]):
            self.start = [x1,y1]
            self.ax.plot(x1,y1,c='r',marker='o',markersize=12)    
        else:
            print("start location is not in free space!")
        if self.is_in_free_space([x2,y2]):
            self.goal = [x2,y2]
            self.ax.plot(x2,y2,c='b',marker='o',markersize=12)    
        else:
            print("goal location is not in free space!")

    def is_collision_free_between(self,q1,q2):
        cutoff = 0.5
        for obstacle in self.obstacles.obstacle_list:
            status, t = obstacle.intersects(q1,q2)
            if status:
                if t > cutoff:
                    t -= cutoff
                else:
                    return True, None # Not collision free, it's just a termination
                qn = [(1-t)*q1[0]+t*q2[0],(1-t)*q1[1]+t*q2[1]]
                return False, qn
        status, t = self.boundaries.intersects(q1,q2)
        if status:
            if t > cutoff:
                t -= cutoff
            else:
                return True, None # Not collision free, it's just a termination
            qn = [(1-t)*q1[0]+t*q2[0],(1-t)*q1[1]+t*q2[1]]                
            return False, qn
        return True, q2

    def is_in_free_space(self,point):
        if not self.boundaries.contains(point[0],point[1]):
            return False
        for obstacle in self.obstacles.obstacle_list:
            if obstacle.contains(point[0],point[1]):
                return False
        return True

    def find_nearest_node(self,qn):
        minDistance = float("inf")
        q_near = None
        for q in self.visited_list:
            currentDistance = np.linalg.norm([q[0]-qn[0],q[1]-qn[1]])
            if minDistance > currentDistance:
                minDistance = currentDistance
                q_near = [q[0],q[1]]
        return q_near

    def in_radius(self,q_new,q_near,radius):
        q1 = np.array(q_near)
        q2 = np.array(q_new)
        d = np.linalg.norm(q1-q2)
        if d <= radius:
            return q_new
        else:
            q2 = (1-radius/d)*q1+(radius/d)*q2
            return q2.tolist()

    def generate_RRT(self):
        if self.start == None or self.goal == None:
            return None
        self.visited_list = {}
        start = self.start
        goal = self.goal
        start_node = Node(start[0],start[1])
        self.visited_list[(start[0],start[1])] = start_node
        distance_to_goal = float("inf")
        while True:
            q_new = np.random.rand(2) * np.array([self.boundaries.lim_xu,self.boundaries.lim_yu,])
            q_new = q_new.tolist()
            if tuple(q_new) in self.visited_list:
                continue
            status = False
            q_near = self.find_nearest_node(q_new)
            q_new = self.in_radius(q_new,q_near,self.radius)
            while not status:
                status, q_new = self.is_collision_free_between(q_near,q_new)
            if q_new == None:
                continue
            new_node = Node(q_new[0],q_new[1],self.visited_list[tuple(q_near)])
            self.ax.plot([q_near[0],q_new[0]],[q_near[1],q_new[1]])
            plt.pause(0.001)
            self.visited_list[tuple(q_new)] = new_node
            distance_to_goal = np.linalg.norm([goal[0]-q_new[0],goal[1]-q_new[1]])
            if distance_to_goal <= self.radius:
                stat,_ =self.is_collision_free_between(goal,q_new)
                if stat:
                    break
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
        self.ax.plot([data[-1,0],self.goal[0]],[data[-1,1],self.goal[1]],'-',c='g')
def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--draw_obstacles",
        type=int,
        default=-1,
        help="use this option to draw your custom obstacles")
    args = parser.parse_args()
    
    boundaries = Boundaries()
    
    if args.draw_obstacles <= 0:
        obstacles = Obstacles()
        obstacle1 = ply.Polygon([[4,3],[7,9],[9,7]],True)
        obstacle2 = ply.Polygon([[14,3],[14,9],[20,17],[25,11],[20,10],[28,4],[18,7]],False)
        obstacles.add_obstacle(obstacle1)
        obstacles.add_obstacle(obstacle2)

        my_robot = Point_robot(obstacles,boundaries)
    else:
        my_robot = Point_robot(None,boundaries,args.draw_obstacles)
    end_node = my_robot.generate_RRT()
    if end_node == None:
        return
    path = my_robot.backtrack(end_node)
    my_robot.plot_path(path)
    plt.show()

if __name__ == '__main__':
    main()