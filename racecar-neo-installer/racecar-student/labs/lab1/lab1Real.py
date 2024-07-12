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
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import cv2 as cv
import numpy as np

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
global TRIG_DEADBAND 
global JOY_DEADBAND 
global state
MANUAL = 0
LINE_FOLLOWING = 1
LANE_FOLLOWING = 2
CONE_FOLLOWING = 3
JOY_DEADBAND = 0.15
TRIG_DEADBAND = 0.1
GREEN = ((35, 127.5, 209.1), (50.5, 204, 155.55)) # The HSV range for the color blue
# BLUE = ((100, 13, 52), (113, 50, 87))  # The HSV range for the color green
BLUE = ((80, 33.15, 132.6), (125, 127.5, 221.85))
HEIGHT = rc.camera.get_height()
WIDTH = rc.camera.get_width()
kP = 0.2
CROP_FLOOR = ((330, 0), (HEIGHT - 30, WIDTH))
MIN_LINE_CONTOUR_AREA = 75
MIN_CONE_CONTOUR_AREA = 1500
MIN_SPEED = 0.3
MAX_SPEED = 1
MIN_ANGLE = -1
MAX_ANGLE = 1
A_RELEASED = True
B_RELEASED = True
B_RELEASED = True

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global state
    global speed_offset
    global angle_offset
    global speed_inc
    global angle_inc

    state = 0 # The initial state
    speed = 0.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle is 0.0

    # This tells the car to begin at a standstill
    rc.drive.stop()

def update_contour_line():
    global contour_center
    global contour_area
    global cone_center
    global cone_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_area = 0;
        contour_center = None;
    else:
         # Find the contour of the line and update contour_center and contour_area
        cropped_image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        # contours = rc_utils.find_contours(cropped_image, BLUE[0], BLUE[1])
        contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
        line_contour = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        # if line_contour is None:
        #     contours = rc_utils.find_contours(cropped_image, cone_color_priority[1][0], cone_color_priority[1][1])
        #     line_contour = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        #     if line_contour is None:
        #         contours = rc_utils.find_contours(cropped_image, cone_color_priority[2][0], cone_color_priority[2][1])
        #         line_contour = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        # Find cone contour
        # contours = rc_utils.find_contours(image, cone_color[0], cone_color[1])
        # cone_contour = rc_utils.get_largest_contour(contours, MIN_CONE_CONTOUR_AREA)

        # Update center and area variables
        if line_contour is not None:
            contour_area = rc_utils.get_contour_area(line_contour)
            contour_center = rc_utils.get_contour_center(line_contour)
            rc_utils.draw_contour(cropped_image, line_contour)
            rc_utils.draw_circle(cropped_image, contour_center)
        else:
            contour_area = 0
            contour_center = None
        
        # Find the contour of the cone and draw its center
        # if cone_contour is not None:
        #     center = rc_utils.get_contour_center(cone_contour)
        #     rc_utils.draw_contour(image, cone_contour)
        #     rc_utils.draw_circle(image, center)
        #     cone_area = rc_utils.get_contour_area(cone_contour)
        #     cone_center = rc_utils.get_contour_center(cone_contour)
        # else:
        #     cone_center = None
        #     cone_area = 0

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        row = 410
        col = 130
        hsvimg = np.zeros((300, 300, 3), np.uint8)
        hsvimg[:] = hsv[row][col]
        BGRimg = cv.cvtColor(hsvimg, cv.COLOR_HSV2BGR)
        # rc_utils.draw_circle(image, (x, y))
        cv.circle(image, (col, row), 1, (30, 255, 255), 1)
        cv.namedWindow("Color", cv.WINDOW_NORMAL)
        cv.imshow("Color", BGRimg)
        print(hsv[row][col])
        rc.display.show_color_image(image)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed
    global angle
    global state
    global A_RELEASED
    global B_RELEASED
    global X_RELEASED
    rc.drive.set_max_speed(1)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A) and A_RELEASED:
        state += 1
        if (state > 1):
            state = 0
        A_RELEASED = False
    elif not rc.controller.is_down(rc.controller.Button.A):
        A_RELEASED = True

    if state == MANUAL:
        # Drive
        (_, xD) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
        # rightJoy = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
        # # rightTrig = rc_utils.remap_range(rightTrig, 0, 1, 0, 0.3)
        # leftJoy = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
        if (xD > JOY_DEADBAND or xD < -JOY_DEADBAND):
            speed = rc_utils.remap_range(xD, -1, 1, -1, 1)
        else:
            speed = 0
        # elif leftTrig > 0.0:
        #     if (leftTrig > TRIG_DEADBAND):
        #         speed = -rc_utils.remap_range(leftTrig, 0, 1, 0, 1)
        #     else:
        #         speed = 0
        # else:
        #     speed = 0
        
        # Steering
        (xS, _) = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
        if (xS > JOY_DEADBAND or xS < -JOY_DEADBAND):
            angle = rc_utils.remap_range(xS, -1, 1, -0.25, 0.25)
        else:
            if rc.controller.is_down(rc.controller.Button.B):# and B_RELEASED:
                angle = 0.1
            # elif not rc.controller.is_down(rc.controller.Button.B):
            #     angle = 0.0
            #     B_RELEASED = True
            
            elif rc.controller.is_down(rc.controller.Button.X):# and X_RELEASED:
                angle = -0.1
            # elif not rc.controller.is_down(rc.controller.Button.X):
            #     angle = 0.0
            #     X_RELEASED = True
            else:
                angle = 0
    elif state == LINE_FOLLOWING:
        # Search for contours in the current color image
        update_contour_line()

        # if (rc.controller.is_down(rc.controller.Button.X) and X_Released):
        #     cone_ind += 1
        #     if (cone_ind >= NUM_CONES):
        #         cone_ind = 0

        #     cone_color = cones[cone_ind][COLOR_IND]
        #     cone_params = cones[cone_ind][PARAMS_IND]
        #     cone_dist = cones[cone_ind][DIST_IND]
        #     cone_color_priority = cones[cone_ind][PRIORITY_IND]
        #     cone_name = cones[cone_ind][NAME_IND]
        #     X_Released = False
        #     print("Targeting cone: " + cone_name)
        # elif not rc.controller.is_down(rc.controller.Button.X):
        #     X_Released = True

        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        # if cone_center is not None:
        #     dist = inverse(cone_area, cone_params[0], cone_params[1])
        # if cone_center is not None and dist < cone_dist:
        #         speed = 0
        if contour_center is not None:
            print("yes")
            setpoint = WIDTH // 2
            error = rc_utils.remap_range(setpoint - contour_center[1], -setpoint, setpoint, -1, 1)

            # time = rc.get_delta_time()
            # integral += error * time
            # deriv = (contour_center[1] - last_readings[-1]) / time
            angle = kP * error# + kI * integral + kD * deriv
            angle = rc_utils.clamp(angle, MIN_ANGLE, MAX_ANGLE)
            speed = 1#max(MAX_SPEED - abs(error), MIN_SPEED)
            # last_readings.append(contour_center[1])
            # times.append(times[-1] + time)
        # elif cone_center is None:
        #     speed = MIN_SPEED

        # Set the speed and angle of the RACECAR after calculations have been complete
    # rc.drive.set_speed_angle(speed, angle)

        # Print the current speed and angle when the A button is held down
        # if rc.controller.is_down(rc.controller.Button.A):
        #     print("Speed:", speed, "Angle:", angle)

        # Print the center and area of the largest contour when B is held down
        # if rc.controller.is_down(rc.controller.Button.B):
            # if contour_center is None:
            #     print("No contour found")
            # else:
            #     print("Center:", contour_center, "Area:", contour_area)
      
    # print(speed)

    # print(x)
    
    # if x > 0.0:
    #     angle = angle_offset
    # elif x < 0.0:
    #     angle = -angle_offset
    # else:
    #     angle = 0

    # if rc.controller.was_pressed(rc.controller.Button.A):
    #     speed_offset = min(speed_offset + speed_inc, 1.0)
    #     print ("The current speed of the race car is: " + str(speed_offset))
    # if rc.controller.was_pressed(rc.controller.Button.B):
    #     speed_offset = max(speed_offset - speed_inc, 0.0)
    #     print ("The current speed of the race car is: " + str(speed_offset))

    # if rc.controller.was_pressed(rc.controller.Button.Y):
    #     angle_offset = min(angle_offset + angle_inc, 1.0)
    #     print ("The current angle of the race car is: " + str(angle_offset))
    # if rc.controller.was_pressed(rc.controller.Button.X):
    #     angle_offset = max(angle_offset - angle_inc, 0.0)
    #     print ("The current angle of the race car is: " + str(angle_offset))

    # Send the speed and angle values to the RACECAR
    print(str(speed) + " " + str(angle))
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()