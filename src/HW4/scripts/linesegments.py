import numpy as np

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
            return True, t
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
                return True, max(ts)    
            else:
                return False, None
        else:
            return False, None
