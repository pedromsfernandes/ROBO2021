#!/usr/bin/env python

# NOTE: IMPORTANT! You need to add execute permissions to the file (chmod +x main.py)

import sys
import rospy
import math
from operator import itemgetter
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Parameters in sensor_msgs LaserScan (NOTE: all floats unless otherwise stated)
# range_min, range_max, min, max, dahead, dleft, dright, ranges([]), angle_min, angle_max, angle_increment, time_increment, scan_time, intensities([])

# NOTE: Twist - linear and angular velocity (both Vector3)

SPEED = 0.2
ANGULAR_SPEED = 0.2

# PD control
direction = 0 # 1 for wall on the right, -1 for wall on the left
rotate_direction = 0
kp = 10
#kd = 2
e_prev = 0
e = 0
ed = 0
min_dist_index = -1 # Index of the ray closest to the wall
min_dist = 0
min_angle = 0
target_dist = 0.21 # Target distance to the wall

state = {
    'FIND': 0, # Wander until wall is found
    'FOLLOW': 1, # Wall was found, follow it
    'ROTATE': 2
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

def process_laser_readings():
    global laser_readings, distances_to_obst, e, min_dist_index, min_dist, target_dist, direction
    num_rays = len(laser_readings['ranges'])

    inc = int(math.floor(180/5)) # Back right and left laser ranges determined manually (30 degrees each)

    distances_to_obst['back_right'] = min(laser_readings['ranges'][0:29])
    distances_to_obst['right'] = min(laser_readings['ranges'][30:30+(1*inc-1)])
    distances_to_obst['front_right'] = min(laser_readings['ranges'][30+(1*inc):30+(2*inc-1)])
    distances_to_obst['front'] = min(laser_readings['ranges'][30+(2*inc):30+(3*inc-1)])
    distances_to_obst['front_left'] = min(laser_readings['ranges'][30+(3*inc):30+(4*inc-1)])
    distances_to_obst['left'] = min(laser_readings['ranges'][30+(4*inc):30+(5*inc-1)])
    distances_to_obst['back_left'] = min(laser_readings['ranges'][30+5*inc:239])

    min_dist = min(laser_readings['ranges'])

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

    e = target_dist - min_dist # Max distance to the wall (max range of the sensor)

def rotate():
    global distances_to_obst, laser_readings, current_state, k, alpha, min_dist_index, direction
    
    if (distances_to_obst['front'] > laser_readings['range_max']):
        current_state = state['FOLLOW']

    msg_to_send = Twist()
    msg_to_send.linear.x = 0
    msg_to_send.angular.z = 2 * direction
    return msg_to_send
    

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
        
    msg_to_send = Twist()
    msg_to_send.linear.x = SPEED
    msg_to_send.angular.z = 0
    return msg_to_send

def follow():
    global distances_to_obst, laser_readings, current_state, k, alpha, min_dist_index, direction, rotate_direction

    max_distance = laser_readings['range_max']
    msg_to_send = None

    rospy.loginfo('max_distance/2: ' + str(max_distance/2))
    rospy.loginfo('distance from front: ' + str(distances_to_obst['front']))

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
    ):

        if (distances_to_obst['front'] < max_distance):
            current_state = state['ROTATE']

        msg_to_send = Twist()

        if (distances_to_obst['front'] < target_dist):
            msg_to_send.linear.x = 0
        elif (distances_to_obst['front'] < target_dist * 2):
            msg_to_send.linear.x = SPEED * 0.25
        else:
            msg_to_send.linear.x = SPEED * 0.5

        msg_to_send.angular.z = direction * (kp * e)
        if (msg_to_send.angular.z < -0.5):
            msg_to_send.angular.z = -0.5
        elif (msg_to_send.angular.z > 0.5):
            msg_to_send.angular.z = 0.5
    else:
        current_state = state['FIND']

    return msg_to_send

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
    global SPEED, ANGULAR_SPEED, current_state, laser_readings, rotate_direction

    rospy.init_node('wall_following', anonymous=True)
    rospy.Subscriber(robot_frame_id + '/' + laser_frame_id, LaserScan, laser_callback)
    publisher = rospy.Publisher(robot_frame_id + '/cmd_vel', Twist, queue_size=1)

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
            publisher.publish(msg_to_send)
        rate.sleep()

if __name__ == '__main__':
    robot = sys.argv[1]
    laser = sys.argv[2]
    try:
        wall_following(robot, laser)
    except rospy.ROSInterruptException:
        pass