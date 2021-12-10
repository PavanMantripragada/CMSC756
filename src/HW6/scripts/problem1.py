import numpy as np
import argparse
import matplotlib.pyplot as plt
import polygons as ply
import linesegments as ls
import heapq

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
        self.actions = ((1,0),(0,1),(-1,0),(0,-1))
        self.cost_to_come = float('inf')
        self.cost_to_goal = float('inf')

    def get_children(self):
        children = []
        for u,v in self.actions:
            x_child = self.x+u
            y_child = self.y+v
            child = Node(x_child,y_child,self)
            child.cost_to_come = self.cost_to_come + abs(u) + abs(v)
            child.cost_to_goal = child.cost_to_come
            children.append(child)
        return children

    def __lt__(self, other):
        return (self.x < other.x)


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
        self.ax.set_title("Please select start and goal!")
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
        self.ax.set_title("Map")

    def _init_map(self):
        self.ax.set_title("Please select start and goal, your selection will snap to the grid!!!",c='g')
        print("Disclaimer :::: The points you enter snap to the grid")
        self.fig.set_size_inches(10, 6)
        self.ax.set_xlabel("X axis")
        self.ax.set_ylabel("Y axis")
        self.ax.set_xticks([i for i in range(self.boundaries.lim_xl,self.boundaries.lim_xu+1)])
        self.ax.set_yticks([i for i in range(self.boundaries.lim_yl,self.boundaries.lim_yu+1)])
        self.ax.grid(which='major',color='k', linestyle='--', linewidth=1,alpha=0.1)
        for obstacle in self.obstacles.obstacle_list:
            for [x1,y1],[x2,y2] in obstacle.edge_list:
                self.ax.plot([x1,x2],[y1,y2],c='k')
        for [x1,y1],[x2,y2] in self.boundaries.boundary_list:
            self.ax.plot([x1,x2],[y1,y2],c='k')            
        self.ax.set_xlim(-1,31)
        self.ax.set_ylim(-1,18)
        plt.pause(0.01)
        [(x1,y1),(x2,y2)] = plt.ginput(n=2,timeout=30)
        x1 = round(x1)
        x2 = round(x2)
        y1 = round(y1)
        y2 = round(y2)
        self.ax.plot(x1,y1,c='r',marker='o',markersize=12) 
        self.ax.plot(x2,y2,c='b',marker='o',markersize=12) 
        if self.is_in_free_space([x1,y1]):
            self.start = [x1,y1]
        else:
            print("you have entered start position as ",(x1,y1))
            print("start location is not in free space!")
            self.ax.set_title("start location is not in free space!",c='r')
        if self.is_in_free_space([x2,y2]):
            self.goal = [x2,y2] 
        else:
            print("you have entered start position as ",(x2,y2))
            print("goal location is not in free space!")
            self.ax.set_title("goal location is not in free space!",c='r')

    def is_collision_free_between(self,q1,q2):
        for obstacle in self.obstacles.obstacle_list:
            status, _ = obstacle.intersects(q1,q2)
            if status:
                return False
        status, _ = self.boundaries.intersects(q1,q2)
        if status:           
            return False
        return True

    def is_in_free_space(self,q):
        if not self.boundaries.contains(q[0],q[1]):
            return False
        for obstacle in self.obstacles.obstacle_list:
            if obstacle.contains(q[0],q[1]):
                return False
        return True

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
        #self.ax.plot([data[-1,0],self.goal[0]],[data[-1,1],self.goal[1]],'-',c='g')

    def search_Astar(self):
        if self.start == None or self.goal == None:
            return None        
        status = 0
        visited_list = {}
        open_list = []
        start_node = Node(self.start[0],self.start[1])
        start_node.cost_to_come = 0
        start_node.cost_to_goal = abs(self.goal[0]-start_node.x) + \
                                  abs(self.goal[1]-start_node.y)
        heapq.heappush(open_list,(start_node.cost_to_goal,start_node))
        while len(open_list):
            (_,new_node) = heapq.heappop(open_list)
            #self.ax.plot(new_node.x,new_node.y,marker='o',c='k')
            #plt.pause(0.001)
            if new_node.x == self.goal[0] and new_node.y == self.goal[1]:
                status = 1
                break
            if (new_node.x,new_node.y) in visited_list:
                continue
            visited_list[(new_node.x,new_node.y)] = True
            children = new_node.get_children()
            for child_node in children:
                q1 = [child_node.parent.x,child_node.parent.y]
                q2 = [child_node.x,child_node.y]
                if self.is_collision_free_between(q1,q2):
                    child_node.cost_to_goal += abs(self.goal[0]-child_node.x) + \
                                               abs(self.goal[1]-child_node.y)
                    heapq.heappush(open_list,(child_node.cost_to_goal,child_node))
                    #self.ax.plot(child_node.x,child_node.y,marker='o',c='k',alpha=0.2)
                    #plt.pause(0.001)
        self.ax.set_title("Map",c='k')
        if status:
            print("Yayy path found :)")
            print("Cost of path ::::: ",new_node.cost_to_come)
            return new_node
        else:
            print("Sorry no path found :(")
            return None

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
    end_node = my_robot.search_Astar()
    if end_node != None:
        path = my_robot.backtrack(end_node)
        my_robot.plot_path(path)
    plt.show()

if __name__ == '__main__':
    main()