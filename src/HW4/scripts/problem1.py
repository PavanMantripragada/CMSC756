import numpy as np
import matplotlib.pyplot as plt

class convex_polygon:
    def __init__(self,vertices):
        self.vertices = vertices
        self.normals = self.update_normals()

    def update_normals(self):
        normals = []
        for i in range(len(self.vertices)-1):
            x = self.vertices[i][1] - self.vertices[i+1][1]
            y = self.vertices[i+1][0] - self.vertices[i][0]
            normals.append([x,y])
        x = self.vertices[-1][1] - self.vertices[0][1]
        y = self.vertices[0][0] - self.vertices[-1][0]
        normals.append([x,y])
        return normals

    def project(self,axis):
        projections = []
        for vertex in self.vertices:
            projections.append(vertex[0]*axis[0]+vertex[1]*axis[1])
        start = min(projections)
        end = max(projections)
        return [start,end]
    
    def draw(self,ax):
        coords = self.vertices
        coords.append(coords[0])
        xs,ys = zip(*coords)
        ax.plot(xs,ys,c='k')


def check_overlap(projection1,projection2):
    if projection1[0] <= projection2[1] and projection1[1] >= projection2[0]:
        return True
    else:
        return False

def collison_SAT(polygon1,polygon2):
    collision = True
    for normal in polygon1.normals:
        projection1 = polygon1.project(normal)
        projection2 = polygon2.project(normal)
        if not check_overlap(projection1,projection2):
            return False
    for normal in polygon2.normals:
        projection1 = polygon1.project(normal)
        projection2 = polygon2.project(normal)
        if not check_overlap(projection1,projection2):
            return False
    return True

def main():

    # vertices1 = [[5,7.001],[7,7.001],[6,9.001]]
    # vertices2 = [[4,4],[4,7],[7,7],[7,4]]

    # vertices1 = [[5,7],[7,7],[6,9]]
    # vertices2 = [[4,4],[4,7],[7,7],[7,4]]    
    
    # vertices1 = [[1,1],[3,1],[2,3]]
    # vertices2 = [[4,4],[4,7],[7,7],[7,4]]
    
    vertices1 = [[1,1],[3,1],[2,3]]
    vertices2 = [[4,4],[4,7],[7,7],[7,4]]
    
    polygon1 = convex_polygon(vertices1)
    polygon2 = convex_polygon(vertices2)
    
    if collison_SAT(polygon1,polygon2):
        output = "The polygons are colliding"
        print(output)
    else:
        output = "The polygons are not colliding"
        print(output)

    fig, ax = plt.subplots()
    polygon1.draw(ax)
    polygon2.draw(ax)
    ax.set_title(output)
    plt.show()

if __name__ == '__main__':
    main()