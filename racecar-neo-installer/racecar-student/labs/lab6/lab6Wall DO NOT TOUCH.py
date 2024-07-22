"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import numpy as np
import copy
########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    pass  # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
speed=0
detecting=False
decel=False
accel=0
pre=0
setpoint=0
angle=0
error=0
last_front_dist=1e9
def update():
    global detecting
    global decel
    global speed
    global accel
    global error
    global angle
    global setpoint
    global last_front_dist
    scan = rc.lidar.get_samples()
    angleRF,rightF=rc_utils.get_lidar_closest_point(scan,(10,60))      
    angleLF,leftF=rc_utils.get_lidar_closest_point(scan,(300,350))
    
    right=abs(np.cos(np.radians(angleRF))*rightF)
    left=abs(np.cos(np.radians(angleLF))*leftF)
    # print(str(left) + " " + str(right))
    ang = 0.3
    wall_dist = 30
    if (right < wall_dist and left < wall_dist):
        if left < right:
            angle = ang
        elif right < left:
            angle = -ang
    elif left<wall_dist:
        angle=ang
    elif left>wall_dist:
        angle=-ang
    else:
        angle=0
   

    lid = copy.deepcopy(rc.lidar.get_samples())
    if (len(lid) == 0):
        return
    
    lid_left = lid[int(353*len(lid)//360):]
    lid_right = lid[:int(7*len(lid)//360)]
    left_min = np.max(lid_left)
    right_min = np.max(lid_right)
    dist = max(left_min, right_min)
    print(dist)

    # if dist < 5:
    #     dist = 1e9

    # stop_dist = 250
    # correction = rc_utils.clamp(-2 + (dist / (stop_dist/2)), -2, 0)
    # if speed > 0:
    #     speed = 1 + correction
    # elif speed < 0 and dist > 30:
    #     speed = 0.5
    # elif dist > stop_dist:
    #     speed = 1
    if dist < 1:
        dist = 1e9
    else:
        last_front_dist = dist
    if dist < 55 or last_front_dist < 55:
        speed = 0
    else:
        speed = 0.6
    rc.drive.set_speed_angle(speed, rc_utils.clamp(angle,-1,1))

    



      # Remove 'pass' and write your source code for the update() function here


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
   
    global speed
    global error
    global setpoint
    global angle
    print("angle",angle)
 
    
      # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
