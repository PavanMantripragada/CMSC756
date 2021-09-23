#!/usr/bin/python3.6
import numpy as np
import argparse

def get_pos(time,theta):
    pos = np.around([5*np.cos(theta)+time*np.sin(theta)-time,
                   -5*np.sin(theta)+time*np.cos(theta)],2)

    return pos
def get_orien(theta):
    pos = np.around([np.sin(theta),
                    np.cos(theta)],2)
    return pos
def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--theta",
        help="Input the angle of robot in degrees")
    args = parser.parse_args()

    if args.theta is not None:
        print(f"Input angle is {args.theta} degrees\n")
        theta = np.radians(args.theta)
        orien = get_orien(theta)
        for t in range(6):
            pos = get_pos(t,theta)
            print(f"Position: {pos}, Orientation: {orien} at {t} secs")
    else:    
        thetas = [0,30,45,60,90]
        for theta in thetas:
            orien = get_orien(np.radians(theta))
            print(f"Robot angle: {theta} degs")
            for t in range(6):
                pos = get_pos(t,np.radians(theta))
                print(f"Position: {pos}, Orientation: {orien} at {t} secs")
            

if __name__ == "__main__":
    main()    