# Homework 4
Code for collision detection between 2D polygons and path planning
for point robot

  ## Dependencies to run the codes    
  1. Python 3.6 should be installed on your system.
  2. numpy - Install it using `python3 -m pip install numpy`
  3. argparse - Install it using `python3 -m pip install argparse`
  4. collections - Install it using `python3 -m pip install collections`

  ## Instructions to run the code
  1. Clone the repository by clicking [here!](https://github.com/DrKraig/CMSC756/) 
  2. Open command prompt or terminal.
  3. Navigate to this directory using `cd CMSC756/src/HW4/scripts`
  4. To Run problem 1. If OS is Ubuntu, type `python3 problem1.py`
  5. To Run problem 1. If OS is Windows, type `python problem1.py`
  6. To Run problem 2. If OS is Ubuntu, type `python3 problem2.py`
  7. To Run problem 2. If OS is Windows, type `python problem2.py`
  8. Enjoy!

  ### Specific instructions for Problem 1
  1. You can input the vertices of polygons by editing the code.
  2. Go to line 57-58, which is at the beginning of main().
  3. The variables `vertices1, vertices2` are list of vertices of each polygon.
  4. Each vertex in the list is an ordered pair of `[x,y]`.
  5. The order of the vertices matter while entering. Incorrect order might result in a self-intersecting polygon.
  6. This code doesn't check for convexity of the polygon. So, please only enter convex polygons.
  7. For testing various cases you can use predefined cases in lines 60-67 by uncommenting the respective case.

  Sample run command:
  `python3 problem1.py`

  ### Arguments for Problem 2
  1. --start - coordinates of start   
  2. --goal  - coordinates of goal

  Sample run command:
  `python3 problem2.py --start 5 5 --goal 10 13`

