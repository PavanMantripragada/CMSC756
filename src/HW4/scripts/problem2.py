import argparse
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.actions = ((1,0),(0,1),(-1,0),(0,-1))
    
    def __eq__(self,other):
        if self.x == other.x  and self.y == other.y:
            return 1
        else:
            return 0

    def get_children(self):
        children = []
        for u,v in self.actions:
            x_child = self.x+u
            y_child = self.y+v
            S = np.linspace(0,1,100)
            collision = False
            for s in S:
                x_middle = s*self.x + (1-s)*x_child
                y_middle = s*self.y + (1-s)*y_child
                if not is_in_free_space(x_middle,y_middle):
                    collision = True
                    break
            if not collision:
                child = Node(x_child,y_child)
                child.parent = self
                children.append(child)
        return children


class Graph:
    def __init__(self,root):
        self.root = root

    def search(self,goal):
        status = 0
        visited_list = {}
        open_list = deque()
        open_list.append(self.root)
        while(len(open_list)):
            new_node = open_list.popleft()
            if new_node == goal:
                status = 1
                break
            if (new_node.x,new_node.y) in visited_list:
                continue
            visited_list[(new_node.x,new_node.y)] = True
            children = new_node.get_children()
            for child_node in children:
                open_list.append(child_node)
        if status:
            print("Yayy path found :)")
            return self.print_path(new_node)
        else:
            print("Sorry no path found :(")
            return

    def print_path(self,goal_node):
        node = goal_node
        path = []
        while(node != self.root):
            path.append([node.x,node.y])
            node = node.parent
        path.append([node.x,node.y])
        path.reverse()
        print("Reached goal in steps : ",len(path)-1)
        print(path)
        return path


def is_in_free_space(x,y):
    polygons = []
    polygons.append([[(4,7),(7,13)],[(7,9),(13,11)],[(9,4),(11,7)]])
    polygons.append([[(15,15),(4,10)],[(15,21),(10,11)],[(21,19),(11,8)],[(19,15),(8,4)]])
    polygons.append([[(15,21),(10,18)],[(21,26),(18,12)],[(26,21),(12,11)],
                     [(21,15),(11,10)]])
    polygons.append([[(19,21),(8,11)],[(21,29),(11,5)],[(29,19),(5,8)]])
    #polygons.append([[(15,15),(4,10)],[(15,21),(10,18)],[(21,26),(18,12)],
    #            [(26,21),(12,11)],[(21,29),(11,5)],[(29,19),(5,8)],[(19,15),(8,4)]])
    if x<=0 or x >=35:
        return 0
    if y<=0 or y>=21:
        return 0    
    if (x-4)**2+(y-4)**2>=16 and x<=4 and y<=4:
        return 0
    if (x-4)**2+(y-17)**2>=16 and x<=4 and y>=17:
        return 0
    if (x-31)**2+(y-17)**2>=16 and x>=31 and y>=17:
        return 0
    if (x-31)**2+(y-4)**2>=16 and x>=31 and y<=4:
        return 0
    #i = 0
    for polygon in polygons:
        #print(i)
        count1 = 0
        count2 = 0
        for edge_x,edge_y in polygon:
            s = (y-edge_y[1])/(edge_y[0]-edge_y[1])
            x_intercept = s*edge_x[0]+(1-s)*edge_x[1] 
            if s > 0.0:
                if s<=1.0:
                    if x_intercept > x:
                        count1 +=1
                        #print(edge_x,edge_y)
                    elif x_intercept < x:
                        count2 +=1
                    else:
                        return 0    
        if count1 % 2 != 0 and count2 % 2 != 0:
            return 0
        #i += 1
    return 1

def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--start",
        type=float,
        nargs=2,
        default=[10,10],
        help="coordinates of start")
    parser.add_argument(
        "--goal",
        type=float,
        nargs=2,
        default=[25,10],
        help="coordinates of goal")
    args = parser.parse_args()

    if not is_in_free_space(args.start[0],args.start[1]):
        print("Please select another start location")
        return
    if not is_in_free_space(args.goal[0],args.goal[1]):
        print("Please select another goal location")
        return

    start = Node(args.start[0],args.start[1])
    goal = Node(args.goal[0],args.goal[1])
    route = Graph(start)
    path = np.array(route.search(goal))

    fig = plt.figure()
    polygons = []
    polygons.append([[(4,7),(7,13)],[(7,9),(13,11)],[(9,4),(11,7)]])
    polygons.append([[(15,15),(4,10)],[(15,21),(10,18)],[(21,26),(18,12)],
               [(26,21),(12,11)],[(21,29),(11,5)],[(29,19),(5,8)],[(19,15),(8,4)]]) 
    plt.plot(path[:,0],path[:,1],'-o',c='k')
    plt.plot(args.start[0],args.start[1],'ro')
    plt.plot(args.goal[0],args.goal[1],'go') 
    for polygon in polygons:
        for x,y in polygon:
            plt.plot(x,y,c='k')
    xl = np.linspace(0,35,36)
    yl = np.linspace(0,21,22)
    xs = []
    ys = []
    for x in xl:
        for y in yl:
            xs.append(x)
            ys.append(y)
    plt.scatter(xs,ys,s=5,alpha=0.1)
    plt.show()
if __name__ == '__main__':
    main()
