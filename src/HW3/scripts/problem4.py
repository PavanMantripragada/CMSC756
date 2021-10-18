import numpy as np
import argparse

def line_collision(l1,l2):
    # Collision is checked using cramer's rule
    A = np.array([l1[:2],l2[:2]])
    
    # Check if lines are not parallel
    if abs(np.linalg.det(A)) >= 1e-10:
        return 1

    # If parallel
    else:
        B = np.array([[l1[0],l1[2]],[l2[0],l2[2]]])
        C = np.array([[l1[1],l1[2]],[l2[1],l2[2]]])

        # Check if they have infinite solutions
        if abs(np.linalg.det(B)) <= 1e-10 and abs(np.linalg.det(C)) <= 1e-10:
            return 1
        
        # Else return no collision    
        else:
            return 0

def line_segement_collision(a,b,c,d):
    # Line segment 1 will be (a,b)
    # Line segment 2 will be (c,d)
    # Collision is checked by finding
    # a common point 'p' on vectors a->b & c->d
    # if 0 <= a->p <= ||a->b|| and 0 <= c->p <= ||c->d||
    # collision is detected

    l1,l2 = np.zeros(3),np.zeros(3)
    
    # Angles of vectors a->b and c->d
    theta1 = np.arctan2(b[1]-a[1],b[0]-a[0])
    theta2 = np.arctan2(d[1]-c[1],d[0]-c[0])
    # distance between (a,b) and (c,d)
    r1_max = np.linalg.norm([b[0]-a[0],b[1]-a[1]])
    r2_max = np.linalg.norm([d[0]-c[0],d[1]-c[1]])
    
    # Solving for common point on the vectors
    l1[0] = np.cos(theta1)
    l2[0] = np.sin(theta1)
    l1[1] = -np.cos(theta2)
    l2[1] = -np.sin(theta2)
    l1[2] = c[0]-a[0]   
    l2[2] = c[1]-a[1]
    D = np.linalg.det(np.array([l1[:2],l2[:2]]))  
    D1 = np.linalg.det(np.array([[l1[2],l1[1]],[l2[2],l2[1]]])) 
    D2 = np.linalg.det(np.array([[l1[0],l1[2]],[l2[0],l2[2]]]))
    
    # If the a single common point exists 
    if abs(D) >= 1e-10:
        r1 = D1/D
        r2 = D2/D
        
        # Checking if the common point lies beyond both the segments
        if r1 >= 0.0 and r1<=r1_max and r2 >= 0.0 and r2<=r2_max:
            return 1
        else:
            return 0
    else:
        # Check if there exists multiple common points
        if abs(D1) <= 1e-10 and abs(D2) <= 1e-10:
            rac = np.linalg.norm([c[0]-a[0],c[1]-a[1]])
            rad = np.linalg.norm([d[0]-a[0],d[1]-a[1]])
            rbc = np.linalg.norm([c[0]-b[0],c[1]-b[1]])
            rbd = np.linalg.norm([d[0]-b[0],d[1]-b[1]])
            r = max(rac,rad,rbc,rbd)
            # Checking if any common points lie in the segments
            if r <= r1_max+r2_max:
                return 1    
            else:
                return 0
        else:
            return 0

def contruct_line(a,b):
    # constructing line of form
    # l[0]x+l[1]y+l[2]=0 from points a,b
    l = np.zeros(3)
    l[0] = a[1]-b[1]
    l[1] = b[0]-a[0]
    l[2] = (b[1]-a[1])*a[0] - (b[0]-a[0])*a[1]
    return l


def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--a",
        type=float,
        nargs=2,
        default=[0,0],
        help="coordinates of point a")
    parser.add_argument(
        "--b",
        type=float,
        nargs=2,
        default=[0,1],
        help="coordinates of point b")
    parser.add_argument(
        "--c",
        type=float,
        nargs=2,
        default=[0,-2],
        help="coordinates of point c")
    parser.add_argument(
        "--d",
        type=float,
        nargs=2,
        default=[2,3],
        help="coordinates of point d")    
    args = parser.parse_args()
    if args.a == args.b:
        print("Can't create a line with points",args.a,args.b)
        return
    if args.c == args.d:
        print("Can't create a line with points",args.c,args.d)
        return
    
    # you can edit here to overide the user-given arguments 
    a = args.a #[1,1]
    b = args.b #[1,-1]
    c = args.c #[-1,-1]
    d = args.d #[1,-1]  
    
    # constructing both the line equations 
    l1 = contruct_line(a,b)
    l2 = contruct_line(c,d)
    
    # Checking for collision between the lines
    if line_collision(l1,l2):
        print("The lines collide")
    else:
        print("The lines are not colliding")

    # Checking for collision between the line segements    
    if line_segement_collision(a,b,c,d):
        print("The line segements collide")
    else:
        print("The line segments are not colliding")

if __name__ == '__main__':
    main()