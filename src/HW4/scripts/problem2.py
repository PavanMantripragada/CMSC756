import argparse
import numpy as np
from collections import deque
import polygons as ply
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

    def get_children(self,free_space):
        children = []
        for u,v in self.actions:
            x_child = self.x+u
            y_child = self.y+v
            S = np.linspace(0,1,100)
            collision = False
            for s in S:
                x_middle = s*self.x + (1-s)*x_child
                y_middle = s*self.y + (1-s)*y_child
                if not free_space.contains(x_middle,y_middle):
                    collision = True
                    break
            if not collision:
                child = Node(x_child,y_child)
                child.parent = self
                children.append(child)
        return children


class Graph:
    def __init__(self,root,free_space):
        self.root = root
        self.free_space = free_space

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
            children = new_node.get_children(self.free_space)
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

class FreeSpace:
    def __init__(self,polygons):
        self.polygons = polygons

    def contains(self,x,y):
        if x<=0 or x >=35:
            return False
        if y<=0 or y>=21:
            return False   
        if (x-4)**2+(y-4)**2>=16 and x<=4 and y<=4:
            return False
        if (x-4)**2+(y-17)**2>=16 and x<=4 and y>=17:
            return False
        if (x-31)**2+(y-17)**2>=16 and x>=31 and y>=17:
            return False
        if (x-31)**2+(y-4)**2>=16 and x>=31 and y<=4:
            return False
        for polygon in self.polygons:
            if polygon.contains(x,y):
                return False
        return True

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
        default=[27,15],
        help="coordinates of goal")
    args = parser.parse_args()

    polygons = []
    polygons.append(ply.Polygon([[4,7],[7,13],[9,11]],True))
    polygons.append(ply.Polygon([[15,4],[15,10],[21,18],[26,12],[21,11],[29,5],[19,8]],False))
    
    free_space = FreeSpace(polygons)

    if not free_space.contains(args.start[0],args.start[1]):
        print("Please select another start location")
        return
    if not free_space.contains(args.goal[0],args.goal[1]):
        print("Please select another goal location")
        return

    start = Node(args.start[0],args.start[1])
    goal = Node(args.goal[0],args.goal[1])
    route = Graph(start,free_space)
    path = np.array(route.search(goal))

    fig = plt.figure()

    plt.plot(path[:,0],path[:,1],'-o',c='k')
    #plt.plot(args.start[0],args.start[1],'ro')
    #plt.plot(args.goal[0],args.goal[1],'go') 
    for polygon in polygons:
        for [x1,y1],[x2,y2] in polygon.edge_list:
            plt.plot([x1,x2],[y1,y2],c='k')
    xl = np.linspace(0,35,36)
    yl = np.linspace(0,21,22)
    xs = []
    ys = []
    xc = []
    yc = []
    for x in xl:
        for y in yl:
            if free_space.contains(x,y):
                xs.append(x)
                ys.append(y)
            else:
                xc.append(x)
                yc.append(y)
    plt.scatter(xs,ys,c='g',s=5,alpha=0.5)
    plt.scatter(xc,yc,c='r',s=5,alpha=0.5)
    plt.show()
if __name__ == '__main__':
    main()
