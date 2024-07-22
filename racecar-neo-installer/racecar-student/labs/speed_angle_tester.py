"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab1.py

Title: Lab 1 - RACECAR Controller

Author: Akul Goyal

Purpose: Using a Python script and the data polled from the controller module,
write code to replicate a manual control scheme for the RACECAR. Gain a mastery
in using conditional statements, controller functions and an understanding in the
rc.drive.set_speed_angle() function.

Expected Outcome: When the user runs the script, they are able to control the RACECAR
using the following keys:
- When the right trigger is pressed, the RACECAR drives forward
- When the left trigger is pressed, the RACECAR drives backward
- When the left joystick's x-axis has a value of greater than 0, the RACECAR's wheels turns to the right
- When the left joystick's x-axis has a value of less than 0, the RACECAR's wheels turns to the left
- When the "A" button is pressed, increase the speed and print the current speed to the terminal window
- When the "B" button is pressed, reduce the speed and print the current speed to the terminal window
- When the "X" button is pressed, increase the turning angle and print the current turning angle to the terminal window
- When the "Y" button is pressed, reduce the turning angle and print the current turning angle to the terminal window

Environment: Test your code using the level "Neo Labs > Lab 1: RACECAR Controller".
"""
#HI
########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global speed
global angle
X_RELEASED = True
Y_RELEASED = True
A_RELEASED = True
B_RELEASED = True


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    speed = 01.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle is 0.0

    rc.drive.set_max_speed(1)
    rc.set_update_slow_time(0.5)

    # This tells the car to begin at a standstill
    rc.drive.stop()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global A_RELEASED
    global B_RELEASED
    global Y_RELEASED
    global X_RELEASED

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A) and A_RELEASED:
        speed -= 0.01
        A_RELEASED = False
    elif not rc.controller.is_down(rc.controller.Button.A):
        A_RELEASED = True

    if rc.controller.is_down(rc.controller.Button.Y) and Y_RELEASED:
        speed += 0.01
        Y_RELEASED = False
    elif not rc.controller.is_down(rc.controller.Button.Y):
        Y_RELEASED = True

    if rc.controller.is_down(rc.controller.Button.B) and B_RELEASED:
        angle += 0.01
        B_RELEASED = False
    elif not rc.controller.is_down(rc.controller.Button.B):
        B_RELEASED = True

    if rc.controller.is_down(rc.controller.Button.X) and X_RELEASED:
        angle -= 0.01
        X_RELEASED = False
    elif not rc.controller.is_down(rc.controller.Button.X):
        X_RELEASED = True

    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    print(str(speed) + " " + str(angle))
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()