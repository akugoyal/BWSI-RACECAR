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
import threading

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()

# >> Variables
# Used for the PID controller
# t1 = None
integral = 0
last_readings = [0]
times = [0]

cones = [] # List of cones and their data. Order is color, params, distance, line color priority, and then name.

# Information about the current target cone. Gets updated if the target changes.
cone_ind = 0
cone_color = None
cone_params = None
cone_dist = 0
cone_color_priority = None
cone_name = None
X_Released = True # Used for changing the target cone

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# Used for keeping track of the line and cone contours detected in the image.
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cone_center = None
cone_area = 0

# >> Constants
# Bounds for the racecar speed and angle
MIN_SPEED = 0.3
MAX_SPEED = 1
MIN_ANGLE = -1
MAX_ANGLE = 1

# The smallest contour we will recognize as a valid contour
MIN_LINE_CONTOUR_AREA = 75
MIN_CONE_CONTOUR_AREA = 1500

# Height and width of the image from the color camera
HEIGHT = rc.camera.get_height()
WIDTH = rc.camera.get_width()

# Gains for the P controller
kP = -1
kI = 0
kD = 0

BLUE = ((80, 150, 50), (125, 255, 255)) # The HSV range for the color blue
GREEN = ((40, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((0, 210, 210), (10, 255, 255))  # The HSV range for the color red
WHITE = ((0, 60, 150), (179, 70, 255)) # The HSV range for the color white
YELLOW = ((20, 0, 50), (40, 255, 255)) # The HSV range for the color yellow
PURPLE = ((125, 50, 50), (165, 255, 255)) # The HSV range for the color purple
BLACK = ((0, 50, 0), (179, 255, 56)) # The HSV range for the color black
ORANGE = ((10, 50, 50), (20, 255, 255)) # The HSV range for the color orange
PINK = ((90, 170, 150), (179, 230, 255)) # The HSV range for the color pink

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((330, 0), (HEIGHT - 30, WIDTH))

# Line color priorities
WHITE_COLOR_PRIORITY = (RED, GREEN, BLUE)
YELLOW_COLOR_PRIORITY = (RED, BLUE, GREEN)
BLACK_COLOR_PRIORITY = (GREEN, RED, BLUE)
PURPLE_COLOR_PRIORITY = (BLUE, RED, GREEN)
ORANGE_COLOR_PRIORITY = (BLUE, GREEN, RED)
PINK_COLOR_PRIORITY = (GREEN, BLUE, RED)

# Constants for storing information about the cones
NUM_CONES = 6
NUM_ATTRIBUTES = 5
COLOR_IND = 0
PARAMS_IND = 1
DIST_IND = 2
PRIORITY_IND = 3
NAME_IND = 4

# >> Data
YELLOW_DATA = [(119.09076690673828, 3605.0), (119.09076690673828, 3990.5), (106.42233276367188, 4469.5), 
               (106.42233276367188, 5054.5), (106.42233276367188, 5751.0), (93.76941680908203, 6541.0), 
               (93.76941680908203, 7456.5), (82.58345031738281, 8417.0), (82.58345031738281, 9598.0), 
               (73.06322479248047, 10965.5), (73.06322479248047, 12590.5), (73.06322479248047, 14431.5), 
               (63.33633804321289, 16582.5), (63.33633804321289, 19100.0)]
YELLOW_DIST = 100

PURPLE_DATA = [(56.41253, 23810.5), (56.41253, 26949.5), (50.14364, 28060.0), (50.14364, 28372.0), (43.4857, 28413.0), 
               (43.4857, 28905.0), (43.4857, 30518.0), (35.23148, 32656.0), (35.23148, 35724.5), (28.210056, 39571.0), 
               (28.210056, 45766.5), (21.879967, 55658.0)]
PURPLE_DIST = 60

BLACK_DATA = [(117.08385, 2799.5), (114.23585, 2938.0), (114.23585, 2966.0), (106.67781, 3008.5), (106.67781, 3055.0), 
              (104.67302, 3412.0), (104.67302, 3500.0), (100.96707, 3590.0), (100.96707, 3711.5), (100.96707, 3829.5), 
              (97.7365, 3976.5), (97.7365, 4128.5), (94.14725, 4293.5), (94.14725, 4495.0), (88.301476, 4718.0), 
              (88.301476, 4958.0), (84.95589, 5255.0), (84.95589, 5580.5), (84.95589, 5951.5), (79.08421, 6401.0), 
              (79.08421, 6928.5), (72.34155, 7548.5), (72.34155, 8228.5), (67.820755, 9021.0), (67.820755, 10079.5), 
              (67.820755, 11557.0), (60.660454, 13412.0), (60.660454, 15556.5)]
BLACK_DIST = 70

ORANGE_DATA = [(114.62066, 2615.0), (120.72308, 2652.0), (120.72308, 2656.5), (120.72308, 2675.0), (126.88046, 2712.0), 
               (126.88046, 2744.5), (129.88188, 2803.5), (129.88188, 2862.0), (129.75179, 2944.5), (129.75179, 3030.0), 
               (122.66493, 3136.5), (122.66493, 3247.0), (114.53113, 3398.0), (114.53113, 3547.5), (114.53113, 3715.5), 
               (112.79517, 3923.0), (112.79517, 4149.5), (104.46332, 4396.0), (104.46332, 4699.5), (99.02112, 5041.5), 
               (99.02112, 5443.0), (90.692406, 5920.5), (90.692406, 6432.5), (90.692406, 7093.0), (84.84455, 7933.0), 
               (84.84455, 8919.0), (75.26616, 10179.5), (75.26616, 11715.0), (64.50567, 13684.5), (64.50567, 16278.0), 
               (64.50567, 19629.0)]
ORANGE_DIST = 80

PINK_DATA = [
             (109.04715, 2769.5), (109.04715, 3019.0), (116.435936, 3392.5), (110.74535, 3853.0), (110.74535, 4264.5), 
             (100.01456, 4716.0), (100.01456, 5240.5), (90.04938, 5855.5), (90.04938, 6639.0), (80.79254, 7506.5), 
             (80.79254, 8639.5), (72.09478, 10147.5), (72.09478, 12535.0), (61.814377, 14981.0), (61.814377, 18304.5), 
             (61.814377, 22997.0)]
PINK_DIST = 90

WHITE_DATA = [(69.81981, 5135.5), (65.80033, 7240.5), (65.80033, 9496.0), 
              (60.057293, 11892.5), (60.057293, 14424.5), (54.803596, 17024.5), (54.803596, 19643.0), (54.803596, 22484.5)]
WHITE_DIST = 70

########################################################################################
# Functions
########################################################################################

# def show_graph(x_data, y_data):
#     plt.plot(x_data, y_data)
#     plt.show()

# [FUNCTION] The inverse function computes a / x + b based on the given parameters. 
# Used for calculating the area-distance regressions.
def inverse(x, a, b):
    return a / x + b

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
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
        contours = rc_utils.find_contours(cropped_image, cone_color_priority[0][0], cone_color_priority[0][1])
        line_contour = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        if line_contour is None:
            contours = rc_utils.find_contours(cropped_image, cone_color_priority[1][0], cone_color_priority[1][1])
            line_contour = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

            if line_contour is None:
                contours = rc_utils.find_contours(cropped_image, cone_color_priority[2][0], cone_color_priority[2][1])
                line_contour = rc_utils.get_largest_contour(contours, MIN_LINE_CONTOUR_AREA)

        # Find cone contour
        contours = rc_utils.find_contours(image, cone_color[0], cone_color[1])
        cone_contour = rc_utils.get_largest_contour(contours, MIN_CONE_CONTOUR_AREA)

        # Update center and area variables
        if line_contour is not None:
            contour_area = rc_utils.get_contour_area(line_contour)
            contour_center = rc_utils.get_contour_center(line_contour)
            rc_utils.draw_contour(cropped_image, line_contour)
            rc_utils.draw_circle(cropped_image, contour_center)
        else:
            contour_area = 0
            contour_center = None;
        
        # Find the contour of the cone and draw its center
        if cone_contour is not None:
            center = rc_utils.get_contour_center(cone_contour)
            rc_utils.draw_contour(image, cone_contour)
            rc_utils.draw_circle(image, center)
            cone_area = rc_utils.get_contour_area(cone_contour)
            cone_center = rc_utils.get_contour_center(cone_contour)
        else:
            cone_center = None
            cone_area = 0

        rc.display.show_color_image(image)

# [FUNCTION] Utility function to create a list based on the provided information and 
# the format given by the constants.
def create_info_list(color:list, params:tuple, distance:int, priority:list, name:str):
    l = [None for _ in range(NUM_ATTRIBUTES)]
    l.insert(COLOR_IND, color)
    l.insert(PARAMS_IND, params)
    l.insert(DIST_IND, distance)
    l.insert(PRIORITY_IND, priority)
    l.insert(NAME_IND, name)
    return l

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global cone_color
    global cone_color_priority
    global cone_dist
    global cone_params
    global cone_name
    global cones

    # Ignore all warnings
    warnings.filterwarnings("ignore")

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # >> Compute parameters for all cone colors
    #Yellow
    dists = list(zip(*YELLOW_DATA))[0]
    areas = list(zip(*YELLOW_DATA))[1]
    yellow_params, _ = curve_fit(inverse, areas, dists)
    # Purple
    dists = list(zip(*PURPLE_DATA))[0]
    areas = list(zip(*PURPLE_DATA))[1]
    purple_params, _ = curve_fit(inverse, areas, dists)
    # Black
    dists = list(zip(*BLACK_DATA))[0]
    areas = list(zip(*BLACK_DATA))[1]
    black_params, _ = curve_fit(inverse, areas, dists)
    # Orange
    dists = list(zip(*ORANGE_DATA))[0]
    areas = list(zip(*ORANGE_DATA))[1]
    orange_params, _ = curve_fit(inverse, areas, dists)
    # Pink
    dists = list(zip(*PINK_DATA))[0]
    areas = list(zip(*PINK_DATA))[1]
    pink_params, _ = curve_fit(inverse, areas, dists)
    # White
    dists = list(zip(*WHITE_DATA))[0]
    areas = list(zip(*WHITE_DATA))[1]
    white_params, _ = curve_fit(inverse, areas, dists)


    # >> Add parameters to list of cones
    # Yellow
    cones.append(create_info_list(YELLOW, yellow_params, YELLOW_DIST, YELLOW_COLOR_PRIORITY, "YELLOW"))
    # Purple
    cones.append(create_info_list(PURPLE, purple_params, PURPLE_DIST, PURPLE_COLOR_PRIORITY, "PURPLE"))
    # Black
    cones.append(create_info_list(BLACK, black_params, BLACK_DIST, BLACK_COLOR_PRIORITY, "BLACK"))
    # Orange
    cones.append(create_info_list(ORANGE, orange_params, ORANGE_DIST, ORANGE_COLOR_PRIORITY, "ORANGE"))
    # Pink
    cones.append(create_info_list(PINK, pink_params, PINK_DIST, PINK_COLOR_PRIORITY, "PINK"))
    # White
    cones.append(create_info_list(WHITE, white_params, WHITE_DIST, WHITE_COLOR_PRIORITY, "WHITE"))

    # >> Set cone target parameters for default
    cone_color = cones[cone_ind][COLOR_IND]
    cone_params = cones[cone_ind][PARAMS_IND]
    cone_dist = cones[cone_ind][DIST_IND]
    cone_color_priority = cones[cone_ind][PRIORITY_IND]
    cone_name = cones[cone_ind][NAME_IND]

    # Print start message
    print(
        ">> Lab 4 - Line Follower\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area\n"
        "   X button = switch to next target cone"
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
    global cone_ind
    global cone_color
    global cone_params
    global cone_dist
    global cone_color_priority
    global X_Released
    global cone_name
    global integral
    global last_readings
    global times

    # Search for contours in the current color image
    update_contour()

    if (rc.controller.is_down(rc.controller.Button.X) and X_Released):
        cone_ind += 1
        if (cone_ind >= NUM_CONES):
            cone_ind = 0

        cone_color = cones[cone_ind][COLOR_IND]
        cone_params = cones[cone_ind][PARAMS_IND]
        cone_dist = cones[cone_ind][DIST_IND]
        cone_color_priority = cones[cone_ind][PRIORITY_IND]
        cone_name = cones[cone_ind][NAME_IND]
        X_Released = False
        print("Targeting cone: " + cone_name)
    elif not rc.controller.is_down(rc.controller.Button.X):
        X_Released = True

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if cone_center is not None:
        dist = inverse(cone_area, cone_params[0], cone_params[1])
    if cone_center is not None and dist < cone_dist:
            speed = 0
    elif contour_center is not None:
        setpoint = WIDTH // 2
        error = rc_utils.remap_range(setpoint - contour_center[1], -setpoint, setpoint, -1, 1);

        time = rc.get_delta_time()
        integral += error * time
        deriv = (contour_center[1] - last_readings[-1]) / time
        angle = kP * error + kI * integral + kD * deriv
        angle = rc_utils.clamp(angle, MIN_ANGLE, MAX_ANGLE)
        speed = max(MAX_SPEED - abs(error), MIN_SPEED)
        last_readings.append(contour_center[1])
        times.append(times[-1] + time)
    elif cone_center is None:
        speed = MIN_SPEED;

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
        print("X" * 10 + " (No image) " + "X" * 10 + "Cone: " + cone_name + "\t  Speed: " + str(round(speed, 2)) + 
              "\tAngle " + str(round(angle, 2)))
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : Cone: " + cone_name + "\t  Speed: " + str(round(speed, 2)) + "\tAngle " + 
                  str(round(angle, 2)))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : Cone: " + cone_name + "\t  Speed: " + str(round(speed, 2)) + "\tAngle " + 
                  str(round(angle, 2)))
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()