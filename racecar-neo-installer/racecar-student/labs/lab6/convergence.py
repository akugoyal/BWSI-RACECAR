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
import copy
from numpy import Inf
import math

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()


# Used for the controller
integral = 0
last_error = []
times = []

A_Released = True
Y_Released = True

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels

# >> Constants
# Bounds for the racecar speed and angle
MIN_SPEED = 0.15
MAX_SPEED = 1.0
MIN_ANGLE = -1
MAX_ANGLE = 1

# PID controller configurations
DERIV_METHOD = 1        #0 for last point, 1 for five point stencil
INTEGRAL_WINDOW = 0     #0 to disable
kP_ONE_WALL = -1
kP_TWO_WALL = -1.5
kP = kP_ONE_WALL
kI = 0
kD = 0

leftWall = 0
leftClosestAngle = 0
rightWall = 0
rightClosestAngle = 0
DIST_FROM_WALL = 70
LIDAR_MIN_ANGLE = 10
LIDAR_MAX_ANGLE = 60
LIDAR_MAX_DIST = 400
LIDAR_DELTA_THETA = 10
ctr = 0
k_DISTANCE = 0.6
k_ANGLE = 0.4
stop_dist = 250

########################################################################################
# Functions
########################################################################################
def get_camera_image():
    global camera_frames
    if len(camera_frames) < CAMERA_FRAMES_LAG:
        return None
    
    return camera_frames[0]

def update_camera_frames():
    global camera_frames
    if len(camera_frames) >= CAMERA_FRAMES_LAG:
        camera_frames.pop(0)
    camera_frames.append(rc.camera.get_color_image())

def calcIntegral(time, last_error):
    if len(time) >= (1 + len(last_error)):
        integralAcc = 0
        for i in reversed(range(1, 1 + INTEGRAL_WINDOW)):
            integralAcc += (time[i] - time[i - 1]) * last_error[i]
    return 0

def calcDeriv(error, time):
    global last_error
    if DERIV_METHOD == 0:
        if len(last_error) < 2:
            return 0
        else:
            return (error - last_error[-1]) / time 
    elif DERIV_METHOD == 1:
        if len(last_error) >= 4:
            return (-error + 8 * last_error[-1] - 8 * last_error[-3] + last_error[-4]) / 12.0
        else:
            return 0
    
    return 0

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
# def update_contour():
    global contour_center
    global contour_area
    global cone_center
    global cone_area
    global BLUE_SEEN
    global GREEN_SEEN

    image = get_camera_image()
    # image = rc.camera.get_color_image()

    if image is None:
        contour_area = 0
        contour_center = None
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

        # hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # row = 410
        # col = 130
        # hsvimg = np.zeros((300, 300, 3), np.uint8)
        # hsvimg[:] = hsv[row][col]
        # BGRimg = cv.cvtColor(hsvimg, cv.COLOR_HSV2BGR)
        # # rc_utils.draw_circle(image, (x, y))
        # cv.circle(image, (col, row), 1, (30, 255, 255), 1)
        # cv.namedWindow("Color", cv.WINDOW_NORMAL)
        # cv.imshow("Color", BGRimg)
        # print(hsv[row][col])
        rc.display.show_color_image(image)

# [FUNCTION] Utility function to create a list based on the provided information and 
# the format given by the constants.
# def create_info_list(color:list, params:tuple, distance:int, priority:list, name:str):
    l = [None for _ in range(NUM_ATTRIBUTES)]
    l.insert(COLOR_IND, color)
    l.insert(PARAMS_IND, params)
    l.insert(DIST_IND, distance)
    l.insert(PRIORITY_IND, priority)
    l.insert(NAME_IND, name)
    return l

def update_distance_from_wall():
    global leftWall, rightWall, leftClosestAngle, rightClosestAngle
    N = rc.lidar.get_num_samples()
    distances = copy.deepcopy(rc.lidar.get_samples())
    distances[distances == 0] = Inf
    distances[distances > LIDAR_MAX_DIST] = Inf
    left_values = distances[int(N - LIDAR_MAX_ANGLE * N // 360) : int(N - LIDAR_MIN_ANGLE * N // 360)]
    right_values = distances[int(LIDAR_MIN_ANGLE * N // 360) : int(LIDAR_MAX_ANGLE * N // 360)]
    leftWall = min(left_values)
    rightWall = min(right_values)
    leftWall = leftWall if leftWall != Inf else 0
    rightWall = rightWall if rightWall != Inf else 0
    leftClosestAngle = np.where(left_values==leftWall)[0][0] // (N / 360) if np.isin(leftWall, left_values) else 0.0
    rightClosestAngle = np.where(right_values==rightWall)[0][0] // (N / 360) if np.isin(rightWall, right_values) else 0.0
    # print(distances[N - LIDAR_MAX * 2 : N - LIDAR_MIN * 2])
    # print()

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle

    # Ignore all warnings
    warnings.filterwarnings("ignore")

    # Set initial driving speed and angle
    speed = 1
    angle = 0
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab 4 - Line Follower\n"
        "\n"
        "Controls:\n"
        "   A button = print current speed and angle\n"
        "   B button = print contour center and area\n"
        "   X button = switch to next target cone"
    )

# def compute_ilc_compensation(left, right, angle):
#     global ilc_data

#     for i in range(len(ilc_data)):
#         if ilc_data[i][0] == left and ilc_data[i][1] == right:
#             if i + 1 >= len(ilc_data):
#                 return 0
#             else:
#                 next_err = ilc_data[i + 1][3]
#                 ang = ilc_data[i][2]
#                 ilc_data.pop(i)
#                 return ang - angle + (-0.5 * next_err)
    
#     return 0

# def compute_angle_error():
#     N = rc.lidar.get_num_samples()
#     distances = copy.deepcopy(rc.lidar.get_samples())
#     distances[distances == 0] = Inf
#     distances[distances > LIDAR_MAX_DIST] = Inf
#     left_values = distances[int(N - LIDAR_MAX_ANGLE * N // 360) : int(N - LIDAR_MIN_ANGLE * N // 360)]
#     right_values = distances[int(LIDAR_MIN_ANGLE * N // 360) : int(LIDAR_MAX_ANGLE * N // 360)]
#     left_second_angle = 0
#     left_second_distance = 0
#     right_second_angle = 0
#     right_second_distance = 0
#     c_left_angle = 0
#     c_right_angle = 0

#     if leftWall != 0:
#         if leftClosestAngle + LIDAR_DELTA_THETA > LIDAR_MAX_ANGLE:
#             left_second_angle = math.radians(leftClosestAngle - LIDAR_DELTA_THETA)
#             left_second_distance = left_values[int(math.degrees(left_second_angle) * N // 360)]
#         else:
#             left_second_angle = math.radians(leftClosestAngle + LIDAR_DELTA_THETA)
#             left_second_distance = left_values[int(math.degrees(left_second_angle) * N // 360)]
        
#         if left_second_distance == Inf:
#             left_second_distance = next(val for x, val in enumerate(left_values) if val > 0 and val < Inf)
#             left_second_angle = math.radians(next(i for i, x in enumerate(left_values) if x > 0 and x < Inf))
#             # print(left_second_angle.shape)

#         # print(left_second_angle)
#         # print(left_second_distance)
#         # print(leftWall**2 + left_second_distance**2 - 2 * leftWall * rightWall * math.cos(left_second_angle))
#         c_left = math.sqrt(leftWall**2 + left_second_distance**2 - 2 * leftWall * left_second_distance * math.cos(abs(math.radians(leftClosestAngle) - left_second_angle)))
#         # print(leftWall)
#         # print(left_second_distance)
#         # print(c_left)
#         # print(leftClosestAngle)
#         # print(left_second_angle)
#         # print(leftWall * math.sin(abs(leftClosestAngle - left_second_angle)) / c_left)
#         c_left_angle = np.arcsin(leftWall * math.sin(abs(math.radians(leftClosestAngle) - left_second_angle)) / c_left)
#         # print(c_left_angle)
#         c_left_angle = rc_utils.remap_range(c_left_angle, 0, 90, 0, 1)
    
#     if rightWall != 0:
#         if rightClosestAngle + LIDAR_DELTA_THETA > LIDAR_MAX_ANGLE:
#             right_second_angle = math.radians(rightClosestAngle - LIDAR_DELTA_THETA)
#             right_second_distance = right_values[int(math.degrees(right_second_angle) * N // 360)]
#         else:
#             right_second_angle = math.radians(rightClosestAngle - LIDAR_DELTA_THETA)
#             right_second_distance = right_values[int(math.degrees(right_second_angle) * N // 360)]

#         if right_second_distance == Inf:
#             right_second_distance = next(val for x, val in enumerate(right_values) if val > 0 and val < Inf)
#             right_second_angle = math.radians(next(i for i, x in enumerate(right_values) if x > 0 and x < Inf))

#         print(right_second_angle)
#         print(right_second_distance)
#         print(rightWall**2 + right_second_distance**2 - 2 * rightWall * right_second_distance * math.cos(abs(math.radians(rightClosestAngle) - right_second_angle)))
#         c_right = math.sqrt(rightWall**2 + right_second_distance**2 - 2 * rightWall * right_second_distance * math.cos(abs(math.radians(rightClosestAngle) - right_second_angle)))
#         print(rightWall)
#         print(right_second_distance)
#         print(c_right)
#         print(rightClosestAngle)
#         print(right_second_angle)
#         print(rightWall * math.sin(abs(math.radians(rightClosestAngle) - right_second_angle)) / c_right)
#         c_right_angle = np.arcsin(rightWall * abs(math.radians(rightClosestAngle) - right_second_angle) / c_right)
#         print(c_right_angle)
#         c_right_angle = rc_utils.remap_range(c_right_angle, 0, 90, 0, 1)

#     total = leftWall + rightWall
#     print(right_second_distance)
#     print(c_right_angle)
#     return (leftWall / total) * c_left_angle + (rightWall / total) * c_right_angle
    

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed, angle
    global kP, kI, kD
    global A_Released, Y_Released
    global integral, times, last_error
    global stop_dist
    # global ilc_data
    # print(rc.lidar.get_num_samples())
    lid = copy.deepcopy(rc.lidar.get_samples())
    if (len(lid) == 0):
        return
    # lid[lid == 0] = Inf
    # lid[lid > LIDAR_MAX_DIST] = Inf


    if (rc.controller.is_down(rc.controller.Button.A) and A_Released):
        stop_dist -= 1
        # print(stop_dist)
        A_Released = False
    elif not rc.controller.is_down(rc.controller.Button.A):
        A_Released = True
    
    if (rc.controller.is_down(rc.controller.Button.Y) and Y_Released):
        stop_dist += 1
        # print(stop_dist)
        Y_Released = False
    elif not rc.controller.is_down(rc.controller.Button.Y):
        Y_Released = True
    
    # print(str(lid[0]) + " " + str(lid[int(45 * len(lid) // 360)]) + " " + str(lid[int(90 * len(lid) // 360)]) + " " + str(lid[int(270 * len(lid) // 360)]) + " " + str(lid[int(315 * len(lid) // 360)]))
    lid_left = lid[int(353*len(lid)//360):]
    lid_right = lid[:int(7*len(lid)//360)]
    left_min = np.max(lid_left)
    right_min = np.max(lid_right)
    dist = max(left_min, right_min)
    accel = rc.physics.get_linear_acceleration()[2]
    print(str(dist) + " " + str(stop_dist) + " " + str(speed) + " " + str(angle) + " " + str(accel))
    # print(accel)
    if dist < 5:
        dist = 1e9

    # stop_dist_2 = 100
    # if (dist > stop_dist_2):
    correction = rc_utils.clamp(-2 + (dist / (stop_dist/2)), -2, 0)
    if speed > 0:
        speed = 1 + correction
    elif speed < 0 and dist > 30:
        speed = 0.5
    elif dist > stop_dist:
        speed = 1

    # if (dist < stop_dist_2):
    #     if (accel > 0):
    #         speed = -1
    #     elif accel < 0:
    #         speed = 1
        # speed = 0

    # for i in lidar_results:
    # print(f"Lidar: {dist}")
    # if dist < 130:
    #     rc.drive.stop()
    #     print("Stop")
    #     if accel <= 0:
    #         if dist > 180:
    #             speed = speed
    #         else:
                # speed = -0.2

    angle = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)[0]

    speed = rc_utils.clamp(speed, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_speed_angle(speed, angle)
    return
    update_distance_from_wall()
    update_camera_frames()

    if (rc.controller.is_down(rc.controller.Button.X) and A_Released):
        A_Released = False
        kP += 0.00001
        print(kP)
    elif not rc.controller.is_down(rc.controller.Button.X):
        A_Released = True

    if (rc.controller.is_down(rc.controller.Button.Y) and Y_Released):
        kP -= 0.00001
        print(kP)
        Y_Released = False
    elif not rc.controller.is_down(rc.controller.Button.Y):
        Y_Released = True
        

    if ((leftWall == 0 or rightWall == 0) and not (leftWall == 0 and rightWall == 0)):
        setpoint = DIST_FROM_WALL
        dist_error = leftWall - setpoint if rightWall == 0 else setpoint - rightWall
        kP = kP_ONE_WALL
    elif (leftWall != 0 and rightWall != 0):
        setpoint = (leftWall + rightWall) / 2
        dist_error = leftWall - setpoint
        kP = kP_TWO_WALL
    dist_error = rc_utils.remap_range(dist_error, -100, 100, -1, 1)

    # angle_error = compute_angle_error()
    error = dist_error #+ k_ANGLE * angle_error

    time = rc.get_delta_time()
    integral = integral + error * time if INTEGRAL_WINDOW == 0 else calcIntegral(times, last_error)
    deriv = calcDeriv(error, time)

    angle = kP * error + kI * integral + kD * deriv
    # angle += compute_ilc_compensation(round(leftWall, 0), round(rightWall, 0), angle)
    angle = rc_utils.clamp(angle, MIN_ANGLE, MAX_ANGLE)

    speed = max(MAX_SPEED - 1.25 * abs(error), MIN_SPEED)

    last_error.append(error)
    if (len(last_error) > max(INTEGRAL_WINDOW, 5)):
        last_error.pop(0)
    
    times.append(times[-1] + time if len(times) > 0 else time)
    # ilc_data.append((round(leftWall, 0), round(rightWall, 0), angle, error))
    # plt.plot(times, last_readings, label = "Position")
    # plt.plot(times, [setpoint for _ in range(len(times))], label = "Setpoint")
    # plt.draw()

    dist = copy.deepcopy(rc.lidar.get_samples())
    dist[dist == 0] = Inf
    dist[dist > LIDAR_MAX_DIST] = Inf
    dist = dist[-20:].append(dist[:20])
    min_dist = min(dist)

    if (min_dist) < 40:
        speed = 0
    
    # Set the speed and angle of the RACECAR after calculations have been complete
    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    # print(str(leftWall) + "\t" + str(rightWall) + "\t  Speed: " + str(round(speed, 2)) + "\tAngle " + str(round(angle, 2))
    #        + "\t" + str(compute_ilc_compensation(round(leftWall, 0), round(rightWall, 0), angle)))
    
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    global ctr
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # If an image is found but no contour is found, print all dashes
    # print(str(leftWall) + "\t" + str(rightWall) + "\t  Speed: " + str(round(speed, 2)) + "\tAngle " + str(round(angle, 2)))
    # ctr += 1
    # if (ctr % 4 == 0):
    #     print(ilc_data)
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()