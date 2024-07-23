"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: AR_ID.py

Author: Akul Goyal
"""

'''

AR TAGS IN RACE:
First bend: 1
Tower: 2
Transition to line: 6
Final bend: 5

'''

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import warnings

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():

    # Ignore all warnings
    warnings.filterwarnings("ignore")

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(image)
    for m in markers:
        print(m.get_id())
    image = rc_utils.draw_ar_markers(image, markers)
    rc.display.show_color_image(image)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()