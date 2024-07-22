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
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils
import math
import copy
import numpy as np


########################################################################################
# Global variables
########################################################################################
PI = math.pi; SIN = math.sin; COS = math.cos; ARCSIN = math.asin; ATAN2 = math.atan2; SQRT = math.sqrt; ABS = abs; RAD = math.radians; DEG = math.degrees

rc = racecar_core.create_racecar()
# Constants
CONSTRAINTS = {
    'MAX_INPUT_SPEED': 1,
    'MAX_INPUT_ANGLE': 0.65,
    'MAX_DRIVE_SPEED': 2.0, # meters/second
    'MAX_DRIVE_ANGLE': RAD(32.0) # radians
}
MAX_INPUT_ANGLE = 1
MAX_DRIVE_ANGLE = 32.0 * PI / 180.0
K_t = 2000 # wall correction coefficient
K_f = 0 # wall following coefficient
K_s = 0.5 # stopping coefficient
K_h = 100 # hallway tuning coeff
BETA = PI/2
N_RANGES = 3
SLOW_DOWN_DISTANCE = 250
STOP_DISTANCE = 120

# Variables
samples = []
last_front_dist = 1e9
derivative_sum = 0


########################################################################################
# Functions
########################################################################################


def get_sample(theta):
    '''Gets the distance from a given angle's respective LIDAR sample.'''
    if len(samples)==0:
        return 0
    rad_per_sample = 2*PI/len(samples)
    new_theta = -theta + PI/2
    new_theta = new_theta % (2*PI)
    index = rc_utils.clamp(round(new_theta/rad_per_sample), 0, len(samples)-1)
    return samples[int(index)]


def get_gamma_distance(theta_range):
    '''Gets gamma (wall angle relative to the RACECAR) given the angles of two LIDAR rays, and the LIDAR's distance from the wall.'''
    theta_1, theta_2 = theta_range

    # Calculate wall points
    d_1 = get_sample(theta_1)
    d_2 = get_sample(theta_2)

    if d_1==0 or d_2==0:
        # handle out of range measurements
        return (0, 1e9, 1e9)

    x_1, y_1 = (d_1*COS(theta_1), d_1*SIN(theta_1))
    x_2, y_2 = (d_2*COS(theta_2), d_2*SIN(theta_2))

    # Calculate gamma value, subtract from PI/2 to get difference from RACECAR heading
    dx = x_1 - x_2
    dy = y_1 - y_2
    gamma = PI/2 - ATAN2(dy, dx)
    gamma = (gamma + 2*PI) % (2*PI) -PI

    # Calculate distance to wall
    m = (y_1 - y_2) / (x_1 - x_2)
    A, B, C = (
        m,
        -1,
        -m*x_1 + y_1
    )
    distance = ABS(C) / SQRT(A**2 + B**2)

    distance_sum = d_1+d_2

    return (gamma, distance, distance_sum)


def get_best_wall(intervals):
    '''Finds the most likely closest wall in the given intervals.'''
    gammas = []
    distances = []
    distance_sums = []
    gamma, distance = (0, 1e9)
    for i in range(len(intervals)):
        wall_gamma, wall_distance, wall_distance_sum = get_gamma_distance(intervals[i])

        if wall_distance < 1e3:
            gammas.append(wall_gamma)
            distances.append(wall_distance)
            distance_sums.append(wall_distance_sum)

    # consider each measurement proportional to its distance significance
    distance_sum_reciprocals = [1/i for i in distance_sums]
    distance_sum_reciprocals_sum = sum(distance_sum_reciprocals)
    distance_props = [i/distance_sum_reciprocals_sum for i in distance_sum_reciprocals]
    
    gamma = sum([gammas[i]*distance_props[i] for i in range(len(gammas))])
    distance = sum([distances[i]*distance_props[i] for i in range(len(distances))])
    
    filtered_distance_sums = list(filter((1e9).__ne__, distance_sums))
    if len(filtered_distance_sums)==0:
        filtered_distance_sums = [1e9]

    # squared to weight the farther walls less
    squared_distance_sum = max(filtered_distance_sums)**2

    return (gamma, distance, squared_distance_sum)


def process_LIDAR(verbose=False, left_follow=False, right_follow=False):
    '''Takes in a LIDAR snapshot and returns angle correction values.'''
    global MAX_INPUT_ANGLE, MAX_DRIVE_ANGLE, K_t, BETA, N_RANGES, samples

    ## take in LIDAR snapshot
    samples = rc.lidar.get_samples()

    ## calculate wall values, the clamps are for setting a max error value
    delta_theta = BETA/N_RANGES

    # left
    intervals = [(PI - i*delta_theta, PI - (i+1)*delta_theta) for i in range(N_RANGES)]
    left_gamma, left_distance, left_reciprocal_distance_sum = get_best_wall(intervals)

    # right
    intervals = [(i*delta_theta, (i+1)*delta_theta) for i in range(N_RANGES)]
    right_gamma, right_distance, right_reciprocal_distance_sum = get_best_wall(intervals)

    ## weighting (weight the closer measurement more)
    left_reciprocal_distance_sum = 1/left_reciprocal_distance_sum
    right_reciprocal_distance_sum = 1/right_reciprocal_distance_sum
    reciprocal_sum = left_reciprocal_distance_sum + right_reciprocal_distance_sum

    # multiply by cosine of gamma to give user more control when facing head-on collisions
    left_gamma = rc_utils.clamp(left_gamma, -PI/2, PI/2)
    right_gamma = rc_utils.clamp(right_gamma, -PI/2, PI/2)

    exponent = 1/3
    left_weight = left_reciprocal_distance_sum / reciprocal_sum * COS(left_gamma)**exponent
    right_weight = right_reciprocal_distance_sum / reciprocal_sum * COS(right_gamma)**exponent

    ## calculate correction values
    servo_per_rad = MAX_INPUT_ANGLE / MAX_DRIVE_ANGLE

    # left
    left_denominator = max(1, left_distance**2)
    left_correction = K_t * left_gamma * servo_per_rad / left_denominator
    if not left_follow:
        left_correction = max(0, left_correction)
    left_correction = rc_utils.remap_range(left_correction, -1, 1, -MAX_INPUT_ANGLE, MAX_INPUT_ANGLE)
    left_correction *= left_weight

    # right
    right_denominator = max(1, right_distance**2)
    right_correction = K_t * right_gamma * servo_per_rad / right_denominator
    if not right_follow:
        right_correction = min(0, right_correction)
    right_correction = rc_utils.remap_range(right_correction, -1, 1, -MAX_INPUT_ANGLE, MAX_INPUT_ANGLE)
    right_correction *= right_weight

    # telemetry
    if verbose:
        print(f"LG {round(left_gamma*180/PI, 2)} | LD {round(left_distance, 2)} | LC {round(left_correction, 2)} ||\nRG {round(right_gamma*180/PI, 2)} | RD {round(right_distance, 2)} | RC {round(right_correction, 2)}")
    
    return (left_correction, right_correction)

def get_front_distance():
    '''Returns the average distance of LIDAR samples in front of the RACECAR in a sweep'''
    global samples, last_front_dist

    if len(samples)==0:
        return 1e9

    sweep = PI/12

    distances = []
    rad_per_sample = 2*PI/len(samples)
    n_theta = int(sweep/rad_per_sample)

    theta_start = PI/2 - sweep/2
    for i in range(n_theta):
        sample = get_sample(theta_start + i*rad_per_sample)
        if sample > 5:
            distances.append(sample)

    if len(distances) == 0:
        return 1e9
    
    return min(distances)


def get_distances(theta1, theta2):
    distances = []
    rad_per_sample = 2*PI/len(samples)
    sweep = theta2 - theta1
    n_theta = int(sweep/rad_per_sample)

    for i in range(n_theta):
        sample = get_sample(theta1 + i*rad_per_sample)
        if sample > 5:
            distances.append(sample)
    
    return distances


def get_hallway_correction():
    '''Returns hallway correction value.'''
    beta = PI/3

    D_i = get_front_distance()

    D_r = get_distances(0, beta)
    if len(D_r)==0:
        return 0
    D_r = sum(D_r)/len(D_r)

    D_l = get_distances(PI-beta, PI)
    if len(D_l)==0:
        return 0
    D_l = sum(D_l)/len(D_l)

    D_t = D_l + D_r

    if D_i > 0:
        w_l = D_l / D_t
        w_r = D_r / D_t
        u = K_h * (w_r - w_l) / D_i
        return u
    
    return 0


def start():
    pass


def update():
    global K_t, SLOW_DOWN_DISTANCE, CONSTRAINTS, STOP_DISTANCE, last_front_dist, derivative_sum
    up_button = rc.controller.Button.Y
    down_button = rc.controller.Button.A
    left_button = rc.controller.Button.X
    right_button = rc.controller.Button.B
    l_joy = rc.controller.Button.LJOY
    r_joy = rc.controller.Button.RJOY

    if rc.controller.was_pressed(up_button):
        K_t += 100
    if rc.controller.was_pressed(down_button):
        K_t -= 100

    if rc.controller.was_pressed(left_button):
        SLOW_DOWN_DISTANCE -= 10
    if rc.controller.was_pressed(right_button):
        SLOW_DOWN_DISTANCE += 10

    if rc.controller.was_pressed(l_joy):
        STOP_DISTANCE -= 1
    if rc.controller.was_pressed(r_joy):
        STOP_DISTANCE += 1

    speed, angle = (0,0)

    # Teleop input
    # (lx, ly) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    # speed += ly
    # angle += lx
    speed = 1
    angle = 0
    
    # Angle correction
    left_correction, right_correction = process_LIDAR(verbose=True, left_follow=True, right_follow=True)
    angle += right_correction + left_correction
    
    # Hallway correction
    hallway_correction = get_hallway_correction()
    angle += hallway_correction
    # speed = max(0.6, speed - ABS(hallway_correction)) # decrease speed if sharp turn in hallway

    # Speed correction
    front_dist = get_front_distance()
    speed_correction = rc_utils.clamp((front_dist - SLOW_DOWN_DISTANCE) / SLOW_DOWN_DISTANCE, -max(0, speed), 0)
    speed += K_s * speed_correction
    print(f"FD {round(front_dist, 2)} | Speed {speed} | Angle {angle}")
    print(f"K_t {K_t} | SLOW_DIST {SLOW_DOWN_DISTANCE} | MAX_INPUT_ANGLE {CONSTRAINTS['MAX_INPUT_ANGLE']} | HWC {hallway_correction}")

    # Wall detection
    # lid = copy.deepcopy(rc.lidar.get_samples())
    # if len(lid) == 0:
    #     return
    # lid_left = lid[int(353*len(lid)//360):]
    # lid_right = lid[:int(7*len(lid)//360)]
    # left_max = np.max(lid_left)
    # right_max = np.max(lid_right)
    # dist = max(left_max, right_max)
    # if front_dist < 1:
    #     front_dist = last_front_dist
    # if front_dist < STOP_DISTANCE or last_front_dist < STOP_DISTANCE:
    #     if front_dist > 25:
    #         speed = -1
    #     if front_dist < 25:
    #         speed = 0
    #     angle = 0

    
    if front_dist < 1 or front_dist > 10000:
        front_dist = last_front_dist

    K_d = 0.0002
    derivative = (front_dist - last_front_dist) / rc.get_delta_time()
    derivative_sum += derivative
    speed = max(0, speed + K_d * derivative_sum)
    
    if front_dist < 40:
        speed = 0

    # if front_dist < 200:
    #     speed = -0.5

    

    print(f"dist {front_dist} | deriv sum {derivative_sum}")

    rc.drive.set_speed_angle(
        rc_utils.clamp(speed, -CONSTRAINTS['MAX_INPUT_SPEED'], CONSTRAINTS['MAX_INPUT_SPEED']), 
        rc_utils.clamp(angle, -CONSTRAINTS['MAX_INPUT_ANGLE'], CONSTRAINTS['MAX_INPUT_ANGLE'])
    )
    last_front_dist = front_dist
    derivative_sum *= 0.9
    print()


def update_slow():
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
