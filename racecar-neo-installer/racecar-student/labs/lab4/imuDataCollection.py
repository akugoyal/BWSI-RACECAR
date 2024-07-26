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
import csv

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here


########################################################################################
# Functions
########################################################################################
currentTime=0
xAng=[]
yAng=[]
zAng=[]
xAccel=[]
yAccel=[]
zAccel=[]
time=[]
tireFellTime=0
start=0
record=True
# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    f = open("data.txt", "a")
    f.write("")  # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global currentTime
    global xAng
    global yAng
    global zAng
    global xAccel
    global yAccel
    global zAccel
    global tireFellTime
    global start
    global record

    left_trigger = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    right_trigger = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    left_joystick = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    right_joystick = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)

    ang_vel = rc.physics.get_angular_velocity()
    accel = rc.physics.get_linear_acceleration()
    
    xAng.append(ang_vel[0])
    yAng.append(ang_vel[1])
    zAng.append(ang_vel[2])

    xAccel.append(accel[0])
    yAccel.append(accel[1])
    zAccel.append(accel[2])

    currentTime+=rc.get_delta_time()
    time.append(currentTime)
    if rc.controller.was_pressed(rc.controller.Button.B):
        tireFellTime=currentTime
    if rc.controller.was_pressed(rc.controller.Button.A):
        s=""
        print("xAng",xAng)
        print("yAng",yAng)
        print("zAng",zAng)

        s+="xAng"+str(xAng)+"\n" 
        s+="yAng"+str(yAng)+"\n" 
        s+="zAng"+str(zAng)+"\n" 

        print("xAccel",xAccel)
        print("yAccel",yAccel)
        print("zAccel",zAccel)
        print("tire fell at",tireFellTime)

        s+="xAccel"+str(xAccel)+"\n" 
        s+="yAccel"+str(yAccel)+"\n" 
        s+="zAccel"+str(zAccel)+"\n" 

        s+="time "+str(time)+""+"\n" 
        s+="tire fell at: ("+str(tireFellTime)+")\n" 
        s+="started at: {"+str(start)+"}"

        f = open("data.txt", "w")
        f.write(s)
        f.close()
        
        file_path = 'output2.csv'
        new_data = s
# Append data to CSV file
        with open(file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile,  delimiter=' ', quotechar='|')
            writer.writerows(new_data)
   
    if (right_trigger >0 or left_trigger>0) and record:
        print("jfhdsakjfh")
        start=currentTime
        record=False
        
        
    rc.drive.set_speed_angle(right_trigger - left_trigger, left_joystick[0])
      # Remove 'pass' and write your source code for the update() function here


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
