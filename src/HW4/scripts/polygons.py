import linesegments as ls

class Polygon:
    def __init__(self,vertex_list,convexity):
        self.vertex_list = vertex_list
        self.edge_list = []
        self.vertex_map = {}
        self.convexity = convexity
        self.create_polygon()

    def create_polygon(self):
        for idx in range(len(self.vertex_list)-1):
            self.edge_list.append([self.vertex_list[idx], self.vertex_list[idx+1]])
        self.edge_list.append([self.vertex_list[-1], self.vertex_list[0]])
        for idx in range(len(self.vertex_list)-1):
            if tuple(self.vertex_list[idx]) in self.vertex_map:
                self.vertex_map[tuple(self.vertex_list[idx])].append( \
                    [self.vertex_list[idx-1],self.vertex_list[idx+1]])
            else:
                self.vertex_map[tuple(self.vertex_list[idx])] =  \
                    [[self.vertex_list[idx-1],self.vertex_list[idx+1]]]
        if tuple(self.vertex_list[-1]) in self.vertex_map:
            self.vertex_map[tuple(self.vertex_list[-1])].append( \
                [self.vertex_list[-2],self.vertex_list[0]])
        else:
            self.vertex_map[tuple(self.vertex_list[-1])] =  \
                [[self.vertex_list[-2],self.vertex_list[0]]]
        if not self.convexity:
            self.child_polygon_list = []

    def contains(self,x,y):
        count = 0
        for [x1,y1],[x2,y2] in self.edge_list:
            if y1 == y2:
                if y == y1:
                    if min(x1,x2) <= x and x <= max(x1,x2):
                        return True
            else:        
                s = (y-y2)/(y1-y2)
                x_intercept = s*x1+(1-s)*x2 
                if s > 0.0 and s < 1.0:
                    if x < x_intercept:
                        count +=1
                    elif x == x_intercept:
                        return True
                elif s == 0.0:
                    if x == x_intercept:
                        return True
                    elif x < x_intercept:
                        for [[_,y21],[_,y22]] in self.vertex_map[(x2,y2)]:
                            if (y22-y)*(y21-y) > 0:
                                count +=2
                            else:
                                count +=1
        if count % 2 != 0 :
            return True
        return False

    def intersects(self,q1,q2):
        for p1,p2 in self.edge_list:
            collision_status,t = ls.line_segement_collision(p1,p2,q1,q2)
        return collision_status, t
