"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: lab4.py

Title: Lab 4 - Line Follower

Author: Akul Goyal

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The RACECAR should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, the RACECAR should be fully autonomous
and drive without the assistance of the user. The RACECAR drives according to the following
rules:
- The RACECAR maintains a following behavior by keeping the line in the center of the screen
- The RACECAR's color priority is RED > GREEN > BLUE.
- The angle of the RACECAR is variable, and is calculated after every frame
- The speed of the RACECAR may be static or variable, depending on the programmer's intents
- The RACECAR must adjust to challenges such as ramps, sharp turns, and dead ends

Environment: Test your code using the level "Neo Labs > Lab 4: Line Follower".
Use the "TAB" key to advance from checkpoint to checkpoint to practice each section before
running through the race in "race mode" to do the full course. Lowest time wins!
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import warnings

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()
# >> Data
YELLOW_DATA = [(119.09076690673828, 3605.0), (119.09076690673828, 3990.5), (106.42233276367188, 4469.5), (106.42233276367188, 5054.5), (106.42233276367188, 5751.0), (93.76941680908203, 6541.0), (93.76941680908203, 7456.5), (82.58345031738281, 8417.0), (82.58345031738281, 9598.0), (73.06322479248047, 10965.5), (73.06322479248047, 12590.5), (73.06322479248047, 14431.5), (63.33633804321289, 16582.5), (63.33633804321289, 19100.0)]
yellow_params = None
# x = np.linspace(3500, 19000, 155)
# dists = list(zip(*YELLOW_DATA))[0]
# areas = list(zip(*YELLOW_DATA))[1]
# params, covar = curve_fit(inverse, areas, dists)
# # y = params[0] * np.arctan(params[1] * x + params[2]) + params[3]
# y = params[0] / x + params[1]
# plt.plot(x, y)
# print(f"Params: {params} Covariance: {covar}")
# plt.scatter(areas, dists)
# plt.show()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_LINE_CONTOUR_AREA = 75
MIN_CONE_CONTOUR_AREA = 500
HEIGHT = rc.camera.get_height()
WIDTH = rc.camera.get_width()
kP = -1
MAX_SPEED = 1

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (HEIGHT, WIDTH))

# A crop window for the center of the screen, for the cone
kHeight = 0.25
kWidth = 0.25
CROP_CONE = ((HEIGHT // 2 - int(kHeight * HEIGHT), WIDTH // 2 - int(kWidth * WIDTH)), 
             (HEIGHT // 2 + int(kHeight * HEIGHT), WIDTH // 2 + int(kWidth * WIDTH)))

BLUE = ((80, 150, 50), (125, 255, 255)) # The HSV range for the color blue
GREEN = ((40, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((165, 50, 50), (10, 255, 255))  # The HSV range for the color red
WHITE = ((0, 60, 150), (179, 70, 255)); # The HSV range for the color white
YELLOW = ((20, 0, 50), (40, 255, 255)); # The HSV range for the color yellow
PURPLE = ((125, 50, 50), (165, 255, 255)); # The HSV range for the color purple

# Color priority: Red >> Green >> Blue
COLOR_PRIORITY = (RED, BLUE, GREEN)

# Color of the cone
CONE_COLOR = YELLOW

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cone_center = None;
cone_area = 0;
CONE_STOP_AREA = 5000;
seeingCone = False;

########################################################################################
# Functions
########################################################################################

def inverse(x, a, b):
    try:
        return a / x + b
    except:
        pass

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area
    global cone_center
    global cone_area
    global seeingCone

    image = rc.camera.get_color_image()
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    if image is None:
        contour_area = 0;
        contour_center = None;
    else:
         # Find the contour of the line and update contour_center and contour_area
        cropped_image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])
        contours = rc_utils.find_contours(cropped_image, COLOR_PRIORITY[0][0], COLOR_PRIORITY[0][1])
        line_c = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        if line_c is None:
            contours = rc_utils.find_contours(cropped_image, COLOR_PRIORITY[1][0], COLOR_PRIORITY[1][1])
            line_c = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

            if line_c is None:
                contours = rc_utils.find_contours(cropped_image, COLOR_PRIORITY[2][0], COLOR_PRIORITY[2][1])
                line_c = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        # Find cone contour
        contours = rc_utils.find_contours(image, CONE_COLOR[0], CONE_COLOR[1])
        cone_c = rc_utils.get_largest_contour(contours, MIN_CONE_CONTOUR_AREA)

        # Update center and area variables
        if line_c is not None:
            contour_area = rc_utils.get_contour_area(line_c)
            contour_center = rc_utils.get_contour_center(line_c)
            rc_utils.draw_contour(cropped_image, line_c)
            rc_utils.draw_circle(cropped_image, contour_center)
        else:
            contour_area = 0
            contour_center = None;
        
        # Find the contour of the cone and draw its center
        if cone_c is not None:
            center = rc_utils.get_contour_center(cone_c)
            rc_utils.draw_contour(image, cone_c)
            rc_utils.draw_circle(image, center)
            cone_area = rc_utils.get_contour_area(cone_c)
            cone_center = rc_utils.get_contour_center(cone_c)
            seeingCone = True;
        else:
            cone_center = None
            cone_area = 0
            seeingCone = False;
            # print(cone_area)
            # print(hsv[center[0]][center[1]])

        # row = 390
        # col = 253
        # hsvimg = np.zeros((300, 300, 3), np.uint8)
        # hsvimg[:] = hsv[row][col]
        # BGRimg = cv.cvtColor(hsvimg, cv.COLOR_HSV2BGR)
        # # rc_utils.draw_circle(image, (x, y))
        # cv.circle(image, (col, row), 1, (0, 0, 0), 1)
        # cv.namedWindow("Color", cv.WINDOW_NORMAL)
        # cv.imshow("Color", BGRimg)
        # print(hsv[row][col])

        rc.display.show_color_image(image)

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global yellow_params

    warnings.filterwarnings("ignore")
    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # >>Compute parameters for all cone colors
    #Yellow
    dists = list(zip(*YELLOW_DATA))[0]
    areas = list(zip(*YELLOW_DATA))[1]
    yellow_params, covar = curve_fit(inverse, areas, dists)

    # Print start message
    print(
        ">> Lab 4 - Line Follower\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global kP

    # Search for contours in the current color image
    update_contour()

    # TODO Part 3: Determine the angle that the RACECAR should receive based on the current 
    # position of the center of line contour on the screen. Hint: The RACECAR should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    dist = inverse(cone_area, yellow_params[0], yellow_params[1])
    # if cone_area > CONE_STOP_AREA:
    if dist < 100:
        speed = 0
        angle = 0
    elif contour_center is not None:
        setpoint = WIDTH // 2
        error = rc_utils.remap_range(setpoint - contour_center[1], -setpoint, setpoint, -1, 1);
        angle = kP * error
        speed = max(MAX_SPEED - abs(error), 0.1)
    else:
        angle = 0;
        speed = 0;
        
    # if seeingCone and speed > 0:
        # YELLOW_DATA.append((rc.lidar.get_samples()[0].item(), cone_area))
        # print(rc.lidar.get_samples()[0].item())
        # print(inverse())
        # print(f"Distance: {round(rc.lidar.get_samples()[0].item(), 2)} Area: {round(cone_area, 2)}")
    # Set the speed and angle of the RACECAR after calculations have been complete
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

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10 + " Speed: " + str(round(speed, 2)) + " Angle " + str(round(angle, 2)))
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area) + " Speed: " + str(round(speed, 2)) + " Angle " + str(round(angle, 2)))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area) + " Speed: " + str(round(speed, 2)) + " Angle " + str(round(angle, 2)))
    
    # print(YELLOW_DATA)
    # print()
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()