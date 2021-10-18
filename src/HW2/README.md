# Homework 2
Code for inverse kinematics of spherical elbow manipulator

## Dependencies to run the codes
1. Python 3.6 should be installed on your system.
2. numpy - Install it using `python3 -m pip install numpy`
3. matplotlib - Install it using `python3 -m pip install matplotlib`
4. argparse - Install it using `python3 -m pip install argparse`

## Instructions to run the code
  
1. Clone the repository by clicking [here!](https://github.com/DrKraig/CMSC756)
2. Open command prompt or terminal.
3. Navigate to this directory using `cd CMSC756/src/HW2/scripts`
4. To Run problem 4. If OS is Ubuntu, type `python3 problem4.py`
5. To Run problem 4. If OS is Windows, type `python problem4.py`
6. Enjoy!
    
### Arguments for Problem 4

1. --x - the x position of tool-frame origin
2. --y - the y position of tool-frame origin
3. --z - the z position of tool-frame origin

Sample run command:
`python3 problem4.py --x 1.5 --y 1.5 --z 1`

Note: For ease of entering inputs the R (rotation matrix) is not taken as an argument. You can change that variable, available at line 176. The default value for x,y,z is '1' and R is identity. Robot dimensions can be accesed from line 167-171. 
