# MicroBot

The projects consists of two parts: 
 - Arduino code (pwm_control): That reads position arguments from COM port (X1,X2,X3,X4,X5,X6) where Xi is a integer position between 0 and 180 for motor i. The code forwards the arguments to the PWM module using an i2c connection.
 - Python code (python): A FK model of the robot and a GUI that shows the position given a set joint position arguments.

It is planned to integrate the two parts, to allow the positions from the Python package to be send to the arduino code. Before this is possible several steps should be made. We need to: 
 - Offset the joint values in the python script (typically going from -90 to 90) to fit the robot joints (0 to 180).
 - Calibrate the Servo motors in the arduino script to ensure the position is correctly represented in the 12-bit i2c connection.
 - Provide a UART connection in the python script that sends the positions (which fits the existing COM-port communication protocol)
 - Validate the FK model of the python script to the real robot. This include adjusting the length of the robot links.

  
## Python
To run the python simulation, open a terminal in the python folder. Then run "python GUI_Robot.py". 
Prereq: 
  - Python (3.12.3) 
  - Numpy (2.4.2) 
  - Matplotlib (3.10.8) 
