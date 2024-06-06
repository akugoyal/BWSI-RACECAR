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

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global speed
global angle
global speed_offset
global angle_offset
global speed_inc
global angle_inc

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global speed_offset
    global angle_offset
    global speed_inc
    global angle_inc

    speed = 0.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle is 0.0
    speed_offset = 0.5 # The initial speed offset is 0.5
    angle_offset = 0.5 # The inital angle offset is 0.5
    speed_inc = 0.1 # The initial speed increment is 0.1
    angle_inc = 0.1 # The initial angle increment is 0.1

    # This tells the car to begin at a standstill
    rc.drive.stop()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global speed_offset
    global angle_offset
    global speed_inc
    global angle_inc
    rc.drive.set_max_speed(1)
    # Part 1: Modify the following conditional statement such that when the
    # right trigger is pressed, the RACECAR moves forward at the designated speed.
    # when the left trigger is pressed, the RACECAR moves backward at the designated speed.
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0.0:
        speed = speed_offset
    elif rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0.0:
        speed = -speed_offset
    else:
        speed = 0
      
    # Part 2: Modify the following conditional statement such that when the
    # value of the left joystick's x-axis is greater than 0, the RACECAR's wheels turn right.
    # When the value of the left joystick's x-axis is less than 0, the RACECAR's wheels turn left.
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    if x > 0.0:
        angle = angle_offset
    elif x < 0.0:
        angle = -angle_offset
    else:
        angle = 0

    # Part 3: Write a conditional statement such that when the
    # "A" button is pressed, increase the speed of the RACECAR. When the "B" button is pressed,
    # decrease the speed of the RACECAR. Print the current speed of the RACECAR to the
    # terminal window.
    if rc.controller.was_pressed(rc.controller.Button.A):
        speed_offset = min(speed_offset + speed_inc, 1.0)
        print ("The current speed of the race car is: " + str(speed_offset))
    if rc.controller.was_pressed(rc.controller.Button.B):
        speed_offset = max(speed_offset - speed_inc, 0.0)
        print ("The current speed of the race car is: " + str(speed_offset))

    # Part 4: Write a conditional statement such that when the
    # "X" button is pressed, increase the turning angle of the RACECAR. When the "Y" button 
    # is pressed, decrease the turning angle of the RACECAR. Print the current turning angle 
    # of the RACECAR to the terminal window.
    if rc.controller.was_pressed(rc.controller.Button.Y):
        angle_offset = min(angle_offset + angle_inc, 1.0)
        print ("The current angle of the race car is: " + str(angle_offset))
    if rc.controller.was_pressed(rc.controller.Button.X):
        angle_offset = max(angle_offset - angle_inc, 0.0)
        print ("The current angle of the race car is: " + str(angle_offset))

    # Send the speed and angle values to the RACECAR
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()