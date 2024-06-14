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
# from cone import cone

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

def inverse(x, a, b):
    return a / x + b

cones = [] # List of cones and their data: Yellow, Purple, Black, . Order is color, params, distance, and then priority.
rc = racecar_core.create_racecar()
MIN_SPEED = 0.3

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
# x = np.linspace(2000, 55000, 1000)
# dists = list(zip(*PINK_DATA))[0]
# areas = list(zip(*PINK_DATA))[1]
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
MIN_CONE_CONTOUR_AREA = 1500
HEIGHT = rc.camera.get_height()
WIDTH = rc.camera.get_width()
kP = -1
MAX_SPEED = 1

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((330, 0), (HEIGHT - 30, WIDTH))

# A crop window for the center of the screen, for the cone
kHeight = 0.25
kWidth = 0.25
CROP_CONE = ((HEIGHT // 2 - int(kHeight * HEIGHT), WIDTH // 2 - int(kWidth * WIDTH)), 
             (HEIGHT // 2 + int(kHeight * HEIGHT), WIDTH // 2 + int(kWidth * WIDTH)))

BLUE = ((80, 150, 50), (125, 255, 255)) # The HSV range for the color blue
GREEN = ((40, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((0, 210, 210), (10, 255, 255))  # The HSV range for the color red
WHITE = ((0, 60, 150), (179, 70, 255)); # The HSV range for the color white
YELLOW = ((20, 0, 50), (40, 255, 255)); # The HSV range for the color yellow
PURPLE = ((125, 50, 50), (165, 255, 255)); # The HSV range for the color purple
BLACK = ((0, 50, 0), (179, 255, 56))
ORANGE = ((10, 50, 50), (20, 255, 255))
PINK = ((90, 170, 150), (179, 230, 255))

# Color priority: Red >> Green >> Blue
WHITE_COLOR_PRIORITY = (RED, GREEN, BLUE)
YELLOW_COLOR_PRIORITY = (RED, BLUE, GREEN)
BLACK_COLOR_PRIORITY = (GREEN, RED, BLUE)
PURPLE_COLOR_PRIORITY = (BLUE, RED, GREEN)
ORANGE_COLOR_PRIORITY = (BLUE, GREEN, RED)
PINK_COLOR_PRIORITY = (GREEN, BLUE, RED)

NUM_CONES = 5

# Color of the cone
cone_color = None
cone_params = None
cone_dist = 0
cone_color_priority = None;

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
cone_center = None;
cone_area = 0;
CONE_STOP_AREA = 5000;
seeingCone = False;
X_Released = True;
cone_ind = 0
NUM_ATTRIBUTES = 5;
COLOR_IND = 0;
PARAMS_IND = 1;
DIST_IND = 2;
PRIORITY_IND = 3;
NAME_IND = 4;

########################################################################################
# Functions
########################################################################################

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
            # print("c: " + str(hsv[cone_center[0]][cone_center[1]]))
            seeingCone = True;
        else:
            cone_center = None
            cone_area = 0
            seeingCone = False;
            # print(cone_area)
            # print(hsv[center[0]][center[1]])

        # row = 350
        # col = 170
        # hsvimg = np.zeros((300, 300, 3), np.uint8)
        # hsvimg[:] = hsv[row][col]
        # BGRimg = cv.cvtColor(hsvimg, cv.COLOR_HSV2BGR)
        # # rc_utils.draw_circle(image, (x, y))
        # cv.circle(image, (col, row), 1, (30, 255, 255), 1)
        # cv.namedWindow("Color", cv.WINDOW_NORMAL)
        # cv.imshow("Color", BGRimg)
        # print(hsv[row][col])

        rc.display.show_color_image(image)

def createArr(color:list, params:tuple, distance:int, priority:list, name:str):
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
    global cones

    warnings.filterwarnings("ignore")
    # Initialize variables
    speed = 0
    angle = 0
    cones = []

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


    # >> Add parameters to cones list
    # Yellow
    cones.append(createArr(YELLOW, yellow_params, YELLOW_DIST, YELLOW_COLOR_PRIORITY, "YELLOW"))
    # Purple
    cones.append(createArr(PURPLE, purple_params, PURPLE_DIST, PURPLE_COLOR_PRIORITY, "PURPLE"))
    # Black
    cones.append(createArr(BLACK, black_params, BLACK_DIST, BLACK_COLOR_PRIORITY, "BLACK"))
    # Orange
    cones.append(createArr(ORANGE, orange_params, ORANGE_DIST, ORANGE_COLOR_PRIORITY, "ORANGE"))
    # Orange
    cones.append(createArr(PINK, pink_params, PINK_DIST, PINK_COLOR_PRIORITY, "PINK"))

    # >> Set cone target parameters for default
    cone_color = cones[cone_ind][COLOR_IND]
    cone_params = cones[cone_ind][PARAMS_IND]
    cone_dist = cones[cone_ind][DIST_IND]
    cone_color_priority = cones[cone_ind][PRIORITY_IND]

    # Print start message
    print(
        ">> Lab 4 - Line Follower\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area\n"
        "   Right trigger = next color cone"
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

    # Search for contours in the current color image
    update_contour()

    if (rc.controller.is_down(rc.controller.Button.X) and X_Released):
        cone_ind += 1;
        if (cone_ind >= NUM_CONES):
            cone_ind = 0;

        cone_color = cones[cone_ind][COLOR_IND]
        cone_params = cones[cone_ind][PARAMS_IND]
        cone_dist = cones[cone_ind][DIST_IND]
        cone_color_priority = cones[cone_ind][PRIORITY_IND]
        X_Released = False
        print("Targeting cone: " + cones[cone_ind][NAME_IND])
    elif not rc.controller.is_down(rc.controller.Button.X):
        X_Released = True

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if cone_center is not None:
        dist = inverse(cone_area, cone_params[0], cone_params[1])
        # if cone_area > CONE_STOP_AREA:
        # print(dist < cone_dist)
    if cone_center is not None and dist < cone_dist:
            speed = 0
    elif contour_center is not None:
        setpoint = WIDTH // 2
        error = rc_utils.remap_range(setpoint - contour_center[1], -setpoint, setpoint, -1, 1);
        angle = kP * error
        speed = max(MAX_SPEED - abs(error), MIN_SPEED)
    elif cone_center is None:
        speed = MIN_SPEED;
    # print(speed)
        
    # if seeingCone:
    #     a = rc.lidar.get_samples()
    #     PINK_DATA.append((np.min(a[np.nonzero(a)]), cone_area))
    #     # print(str(np.min(a[np.nonzero(a)])) + " " + str(cone_area))
    #     # print(inverse())
    #     # print(f"Distance: {round(rc.lidar.get_samples()[0].item(), 2)} Area: {round(cone_area, 2)}")
    #     print(PINK_DATA)
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
    
    # print(BLACK_DATA)
    # print()
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()