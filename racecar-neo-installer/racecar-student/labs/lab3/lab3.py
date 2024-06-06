"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab_3.py

Title: Lab 3 - Stoplight Challenge

Author: Akul Goyal

Purpose: Write a script to enable autonomous behavior from the RACECAR. When
the RACECAR sees a stoplight object (colored cube in the simulator), respond accordingly
by going straight, turning right, turning left, or stopping. Append instructions to the
queue depending on whether the position of the RACECAR relative to the stoplight reaches
a certain threshold, and be able to respond to traffic lights at consecutive intersections. 

Expected Outcome: When the user runs the script, the RACECAR should control itself using
the following constraints:
- When the RACECAR sees a BLUE traffic light, make a right turn at the intersection
- When the RACECAR sees an ORANGE traffic light, make a left turn at the intersection
- When the RACECAR sees a GREEN traffic light, go straight
- When the RACECAR sees a RED traffic light, stop moving,
- When the RACECAR sees any other traffic light colors, stop moving.

Considerations: Since the user is not controlling the RACECAR, be sure to consider the
following scenarios:
- What should the RACECAR do if it sees two traffic lights, one at the current intersection
and the other at the intersection behind it?
- What should be the constraint for adding the instructions to the queue? Traffic light position,
traffic light area, or both?
- How often should the instruction-adding function calls be? Once, twice, or 60 times a second?

Environment: Test your code using the level "Neo Labs > Lab 3: Stoplight Challenge".
By default, the traffic lights should direct you in a counterclockwise circle around the course.
For testing purposes, you may change the color of the traffic light by first left-clicking to 
select and then right clicking on the light to scroll through available colors.
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour (Adjust threshold!)
MIN_CONTOUR_AREA = 18000

# TODO Part 1: Determine the HSV color threshold pairs for ORANGE, GREEN, RED, YELLOW, and PURPLE
# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((80, 150, 50), (125, 255, 255))  # The HSV range for the color blue
GREEN = ((30, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED_LOW = ((0, 50, 50), (10, 255, 255))  # The HSV range for the color red
RED_HIGH = ((165, 50, 50), (179, 255, 255))
ORANGE = ((10, 50, 50), (20, 255, 255)) # The HSV range for the color orange
YELLOW = ((20, 50, 50), (30, 255, 255)) # The HSV range for the color yellow
PURPLE = ((125, 50, 50), (165, 255, 255)) # The HSV range for the color purple

# >> Variables
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
startCommand = True
queue = [] # The queue of instructions
stoplight_color = "" # The current color of the stoplight
executingCommand = False

########################################################################################
# Functions
########################################################################################

def isColor(src, color):
    return color[0][0] <= src[0] <= color[1][0] and color[0][1] <= src[1] <= color[1][1] and color[0][2] <= src[2] <= color[1][2]

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area
    global stoplight_color

    image = rc.camera.get_color_image()
    # image = rc_utils.crop(image, (100, 0), (rc.camera.get_height(), rc.camera.get_width()))
    hsv_img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    # print(f"{hsv_img[200][320][0]}, {hsv_img[200][320][1]}, {hsv_img[200][320][2]}")

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found
        # contours = rc_utils.find_contours(hsv_img, BLUE[0], BLUE[1])
        # contours += rc_utils.find_contours(hsv_img, GREEN[0], GREEN[1])
        # contours += rc_utils.find_contours(hsv_img, RED[0], RED[1])
        # contours += rc_utils.find_contours(hsv_img, ORANGE[0], ORANGE[1])
        # contours += rc_utils.find_contours(hsv_img, YELLOW[0], YELLOW[1])
        # contours += rc_utils.find_contours(hsv_img, PURPLE[0], PURPLE[1])
        mask = cv.inRange(hsv_img, BLUE[0], BLUE[1])
        c, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        
        mask = cv.inRange(hsv_img, GREEN[0], GREEN[1])
        c1, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        c += c1

        mask = cv.inRange(hsv_img, RED_LOW[0], RED_LOW[1])
        c1, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        c += c1

        mask = cv.inRange(hsv_img, RED_HIGH[0], RED_HIGH[1])
        c1, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        c += c1

        mask = cv.inRange(hsv_img, ORANGE[0], ORANGE[1])
        c1, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        c += c1

        mask = cv.inRange(hsv_img, YELLOW[0], YELLOW[1])
        c1, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        c += c1

        mask = cv.inRange(hsv_img, PURPLE[0], PURPLE[1])
        c1, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        c += c1
        # contours = (B, G)

        largest_contour = rc_utils.get_largest_contour(c, MIN_CONTOUR_AREA)
        if largest_contour is None:
            contour_center = None
            contour_area = 0
        else:
            contour_center = rc_utils.get_contour_center(largest_contour)
            contour_area = cv.contourArea(largest_contour)

            # TODO Part 3: Repeat the search for all potential traffic light colors,
            # then select the correct color of traffic light detected.
            # if ()
            # print(hsv_img[contour_center[0]][contour_center[1]])
            if (isColor(hsv_img[contour_center[0]][contour_center[1]], BLUE)):
                stoplight_color = "BLUE"
            elif (isColor(hsv_img[contour_center[0]][contour_center[1]], GREEN)):
                stoplight_color = "GREEN"
            elif (isColor(hsv_img[contour_center[0]][contour_center[1]], RED_LOW) or isColor(hsv_img[contour_center[0]][contour_center[1]], RED_HIGH)):
                stoplight_color = "RED"
            elif (isColor(hsv_img[contour_center[0]][contour_center[1]], ORANGE)):
                stoplight_color = "ORANGE"
            else:
                stoplight_color = "OTHER"

            # filtered = []
            # for c in contours:
            #     if cv.contourArea(c) > MIN_CONTOUR_AREA:
            #         filtered.append(c);

            # print(f"{hsv_img[contour_center[0]][contour_center[1]][0]}, {hsv_img[contour_center[0]][contour_center[1]][1]}, {hsv_img[contour_center[0]][contour_center[1]][2]}")

            cv.circle(image, (contour_center[1], contour_center[0]), 4, (255, 255, 255), -1)
            # cv.drawContours(image, c, -1, (0, 0, 0), 2)
            rc_utils.draw_contour(image, largest_contour)
        # Display the image to the screen
        rc.display.show_color_image(image)


# [FUNCTION] The start function is run once every time the start button is pressed
def start():

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(1,0)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)
    rc.get_delta_time()

    # Print start message (You may edit this to be more informative!)
    print(
        ">> Lab 3 - Stoplight Challenge\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global queue
    global startCommand
    global stoplight_color
    global executingCommand

    update_contour()

    # TODO Part 2: Complete the conditional tree with the given constraints.
    if not executingCommand:
        if stoplight_color == "BLUE":
            turnRight()
            # Call the correct function to append the instructions to the list
        elif stoplight_color == "ORANGE":
            turnLeft()
        elif stoplight_color == "GREEN":
            goStraight()
        elif stoplight_color == "RED":
            stopNow()
        stoplight_color = ""
    # Call the correct function to append the instructions to the list

    # TODO Part 3: Implement a way to execute instructions from the queue once they have been placed
    # by the traffic light detector logic (Hint: Lab 2)
    if len(queue) > 0:
        executingCommand = True;
        speed = queue[0][1]
        angle = queue[0][2]
        queue[0][0] -= rc.get_delta_time()
        if queue[0][0] <= 0:
            executingCommand = False;
            queue.pop(0)
    else:
        executingCommand = False;
        speed = 1
        angle = 0


    # Send speed and angle commands to the RACECAR
    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)

# [FUNCTION] Appends the correct instructions to make a 90 degree right turn to the queue
def turnRight():
    global queue

    # TODO Part 4: Complete the rest of this function with the instructions to make a right turn
    queue.append([0.65, 1, 1])

# [FUNCTION] Appends the correct instructions to make a 90 degree left turn to the queue
def turnLeft():
    global queue

    # TODO Part 5: Complete the rest of this function with the instructions to make a left turn
    queue.append([0.56, 1, -1])

# [FUNCTION] Appends the correct instructions to go straight through the intersection to the queue
def goStraight():
    global queue

    # TODO Part 6: Complete the rest of this function with the instructions to go straight
    queue.append([0.25, 1, 0])

# [FUNCTION] Clears the queue to stop all actions
def stopNow():
    global queue
    queue.append([100, 0, 0])

def update_slow():
    print(stoplight_color)
    print(queue)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()