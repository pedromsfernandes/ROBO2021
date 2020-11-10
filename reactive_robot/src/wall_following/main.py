#!/usr/bin/env python

# NOTE: IMPORTANT! You need to add execute permissions to the file (chmod +x main.py)

import sys
import rospy
import math
import random
from operator import itemgetter
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from reactive_robot.msg import DistToWall

SPEED = 0.3 # Variable for linear speed
ANGULAR_SPEED = 2 # Variable for angular speed (used when rotating)

direction = 0 # 1 for wall on the right, -1 for wall on the left
kp = 15 # K constant for the proportional controller
e = 0 # Error used in the proportional controller
min_dist_index = -1 # Index of the ray closest to the wall
min_dist = 0.3 # Closest distance between the wall and the robot (initial value is the max distance of the laser)
min_angle = 0 # Angle of the ray that's closest to the wall
target_dist = 0.19 # Target distance to the wall

random.seed()

state = {
    'FIND': 0, # Wander until wall is found
    'FOLLOW': 1, # Wall was found, follow it
    'ROTATE': 2 # Wall directly in front, rotate according to the position of the last detected wall
}

current_state = state['FIND']

laser_readings = {
    'ranges': [],
    'range_max': 0,
    'range_min': 0,
    'angle_increment': 0,
    'angle_min': 0,
    'angle_max': 0,
    'scan_time': 0,
    'time_increment': 0,
    'intensities': [],
}

distances_to_obst = {
    'back_left': float('inf'),
    'left': float('inf'),
    'front_left': float('inf'),
    'front': float('inf'),
    'front_right': float('inf'),
    'right': float('inf'),
    'back_right': float('inf'),
}

def prep_twist_msg(linear, angular):
    msg = None
    if (linear is not 0 or angular is not 0): 
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
    return msg

def process_laser_readings():
    global laser_readings, distances_to_obst, e, min_dist_index, min_dist, target_dist, direction

    inc = int(math.floor(180/5)) # Back right and left laser ranges determined manually (30 degrees each)
    num_rays = len(laser_readings['ranges'])

    distances_to_obst['back_right'] = min(laser_readings['ranges'][0:29])
    distances_to_obst['right'] = min(laser_readings['ranges'][30:30+(1*inc-1)])
    distances_to_obst['front_right'] = min(laser_readings['ranges'][30+(1*inc):30+(2*inc-1)])
    distances_to_obst['front'] = min(laser_readings['ranges'][30+(2*inc):30+(3*inc-1)])
    distances_to_obst['front_left'] = min(laser_readings['ranges'][30+(3*inc):30+(4*inc-1)])
    distances_to_obst['left'] = min(laser_readings['ranges'][30+(4*inc):30+(5*inc-1)])
    distances_to_obst['back_left'] = min(laser_readings['ranges'][30+5*inc:239])

    min_dist = min(laser_readings['ranges'])

    # Get index (i.e the angle to the start of the range) of the ray that's closest to an object 

    if (min_dist < laser_readings['range_max']):
        min_dist_index = min(enumerate(laser_readings['ranges']), key=itemgetter(1))[0]

        if (min_dist_index <= num_rays/2):
            direction = 1
        else:
            direction = -1
    else:
        min_dist_index = -1
        direction = 0 

    # Calculate error for the proportional component

    e = target_dist - min_dist

def rotate():
    global distances_to_obst, laser_readings, current_state, direction
    
    if (distances_to_obst['front'] > laser_readings['range_max']):
        current_state = state['FOLLOW']

    return prep_twist_msg(0, ANGULAR_SPEED * direction)
    
def find():
    global current_state, distances_to_obst
    max_dist = laser_readings['range_max']

    if (
        distances_to_obst['right'] < max_dist
        or distances_to_obst['front_right'] < max_dist
        or distances_to_obst['front'] < max_dist
        or distances_to_obst['front_left'] < max_dist
        or distances_to_obst['left'] < max_dist
    ):
        current_state = state['FOLLOW']
        
    return prep_twist_msg(SPEED, random.uniform(-0.2, 0.2))

def follow():
    global distances_to_obst, laser_readings, current_state, direction

    max_distance = laser_readings['range_max']
    linear = 0
    angular = 0

    if (
        (
            distances_to_obst['left'] < max_distance
            and distances_to_obst['front_left'] < max_distance
        )
        or
        (
            distances_to_obst['right'] < max_distance
            and distances_to_obst['front_right'] < max_distance
        )
        or distances_to_obst['back_right'] < max_distance
        or distances_to_obst['back_left'] < max_distance
    ):

        if (distances_to_obst['front'] < max_distance):
            current_state = state['ROTATE']

        if (
            (
                distances_to_obst['back_right'] < max_distance
                and distances_to_obst['right'] > max_distance
            )
            or
            (
                distances_to_obst['back_left'] < max_distance
                and distances_to_obst['left'] > max_distance
            )
        ):
            linear = SPEED * 0.25
        else:
            linear = SPEED * 0.5

        angular = direction * (kp * e)
        if (angular < -0.5):
            angular = -0.5
        elif (angular > 0.5):
            angular = 0.5
    else:
        current_state = state['FIND']

    return prep_twist_msg(linear, angular)

def laser_callback(data):
    global laser_readings

    laser_readings['ranges'] = data.ranges
    laser_readings['range_max'] = data.range_max
    laser_readings['range_min'] = data.range_min
    laser_readings['angle_increment'] = data.angle_increment
    laser_readings['scan_time'] = data.scan_time
    laser_readings['time_increment'] = data.time_increment
    laser_readings['angle_min'] = data.angle_min
    laser_readings['angle_max'] = data.angle_max
    laser_readings['intensities'] = data.intensities

    process_laser_readings()

def wall_following(robot_frame_id, laser_frame_id):
    global current_state, min_dist

    rospy.init_node('wall_following', anonymous=True)
    rospy.Subscriber(robot_frame_id + '/' + laser_frame_id, LaserScan, laser_callback)
    twist_publisher = rospy.Publisher(robot_frame_id + '/cmd_vel', Twist, queue_size=1)
    dist_to_wall_publisher = rospy.Publisher('dist_to_wall', DistToWall, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg_to_send = None
        
        if current_state == state['FIND']:
            msg_to_send = find()
        elif current_state == state['FOLLOW']:
            msg_to_send = follow()
        elif current_state == state['ROTATE']:
            msg_to_send = rotate()
        else:
            pass

        if (msg_to_send is not None):
            twist_publisher.publish(msg_to_send)


        dist_to_wall_msg = DistToWall()
        dist_to_wall_msg.dist_to_wall = min_dist
        dist_to_wall_publisher.publish(dist_to_wall_msg)

        rate.sleep()

if __name__ == '__main__':
    robot = sys.argv[1]
    laser = sys.argv[2]
    try:
        wall_following(robot, laser)
    except rospy.ROSInterruptException:
        pass