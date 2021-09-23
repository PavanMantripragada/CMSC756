#!/usr/bin/python3.6
import numpy as np
import argparse
from matplotlib import pyplot as plt

def plot_camera_view(pitch,yaw,speed,duration,initial_height):
    fig,ax = plt.subplots()
    fig.set_size_inches(5, 5)
    plt.axis('equal')
    v = get_velocity(pitch,yaw,speed)
    for t in range(duration+1):
        radius = initial_height + v[2]*t
        center = (v[0]*t,v[1]*t)
        circle = plt.Circle(center, radius, color='g',lw=3,fill=False)
        ax.add_artist(circle)
    plt.title(f"Pitch {round(np.degrees(pitch))} and Yaw {round(np.degrees(yaw))}")
    plt.xlabel("X axis (m)")
    plt.ylabel("Y axis (m)")
    ax.set_xlim(-20,20)
    ax.set_ylim(-20,20)
    return fig

def get_velocity(pitch,yaw,speed):
    v = np.array([[speed],[0],[0]])
    v = Rz(yaw) @ Ry(pitch) @ v
    return v

def Rz(phi):
    Rz = np.identity(3)
    Rz[0][0] = np.cos(phi)
    Rz[0][1] = -np.sin(phi)
    Rz[1][0] = np.sin(phi)
    Rz[1][1] = np.cos(phi)
    return Rz

def Ry(theta):
    Ry = np.identity(3)
    Ry[0][0] = np.cos(theta)
    Ry[0][2] = np.sin(theta)
    Ry[2][0] = -np.sin(theta)
    Ry[2][2] = np.cos(theta)
    return Ry

def main():
    # Argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pitch",
        help="Input the pitch angle of drone in degrees")
    parser.add_argument(
        "--yaw",
        help="Input the yaw angle of drone in degrees")
    parser.add_argument(
        "--save_plots",
        help="Use this arguments to save plots and make sure you plot_path directory available",
        action="store_true")        
    args = parser.parse_args()

    plot_path = "/home/pavan/Documents/Projects/CMSC756/src/HW1/plots/"

    if args.pitch is not None and args.yaw is not None:
        print(f"For pitch: {args.pitch} degrees, yaw: {args.yaw} degrees\n")
        pitch = np.radians(float(args.pitch))
        yaw = np.radians(float(args.yaw))
        speed = 1 # m/s
        initial_height = 5 # m
        duration = 10 # s
        fig = plot_camera_view(pitch,yaw,speed,duration,initial_height)
        if args.save_plots:
            plt.savefig(fname=plot_path+
                        f"cameraview_{round(np.degrees(pitch))}_{round(np.degrees(yaw))}.png",
                        format='png')        
        plt.show()
    else:    
        orientations = [[0,0],[15,45],[-15,90]]
        if args.pitch is not None:
            for i in range(len(orientations)):
                orientations[i][0] = float(args.pitch)
        if args.yaw is not None:
            for i in range(len(orientations)):
                orientations[i][1] = float(args.yaw)
        print(orientations)        
        figures = []        
        for pitch,yaw in orientations:
            pitch = np.radians(pitch)
            yaw = np.radians(yaw)
            speed = 1 # m/s
            initial_height = 5 # m
            duration = 10 # s
            fig = plot_camera_view(pitch,yaw,speed,duration,initial_height)
            if args.save_plots:
                plt.savefig(fname=plot_path+
                            f"cameraview_{round(np.degrees(pitch))}_{round(np.degrees(yaw))}.png",
                            format='png')
            figures.append(fig)
        plt.show()
if __name__ == "__main__":
    main()    