<h1>Homework 1</h1>

  <h2>Dependencies to run the codes:</h2>

    1. Python 3.6 should be installed on your system.
    2. numpy - Install it using 'python3 -m pip install numpy'
    3. matplotlib - Install it using 'python3 -m pip install matplotlib'
    4. argparse - Install it using 'python3 -m pip install argparse'

  <h2>Instructions to run the code:</h2>
  
    1. Clone the repository by clicking the big green button located here: https://github.com/DrKraig/CMSC756
    2. Open command prompt or terminal.
    3. Navigate to this directory using 'cd CMSC756/src/HW1/scripts'
    4. To Run problem 4. If OS is Ubuntu, type 'python3 problem4.py'
    5. To Run problem 4. If OS is Windows, type 'python problem4.py'
    6. To Run problem 5. If OS is Ubuntu, type 'python3 problem5.py'
    7. To Run problem 5. If OS is Windows, type 'python problem5.py'
    8. Enjoy!
    9. All the plots will be saved in /src/HW1/plots
    
<h3>Arguments for Problem 4:</h3>

    1. --theta - the angle in deg. at which the bot is rotated.
    
    Sample run command:
    python3 problem4.py --theta 45

    Note: If no argument is passed the program prints out values for angles [0,30,45,60,90]

<h3>Arguments for Problem 5:</h3>

    1. --pitch - the pitch angle in deg.
    2. --yaw - the yaw angle in deg.
    3. --save_plots - saves the figures generated (before using this argument make sure plot_path variable is set correctly for your system)

    Sample run command:
    python3 problem5.py --yaw 60

    Note: If no arguments are passed the program plots for default angles. pitch - 0, 15, -15 and yaw - 0, 45, 90.


