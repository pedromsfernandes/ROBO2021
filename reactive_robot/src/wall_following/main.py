#!/usr/bin/env python

# IMPORTANT! You need to add execute permissions to the file (chmod +x main.py)

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
k = 0.15
alpha = 0

state = {
    'FIND': 0,
    'GO': 1,
    'FOLLOW': 2
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
    'min_dist_index': -1 # Index of the ray that is closest to the wall
}

def process_laser_readings():
    global laser_readings, distances_to_obst
    num_rays = len(laser_readings['ranges'])
    #inc = int(math.floor(num_rays/6)) # divide laser into 6 sections; floor() return a float in Python 2.X h
    inc = int(math.floor(180/5))

    distances_to_obst['back_right'] = min(laser_readings['ranges'][0:29])
    distances_to_obst['right'] = min(laser_readings['ranges'][30:30+(1*inc-1)])
    distances_to_obst['front_right'] = min(laser_readings['ranges'][30+(1*inc):30+(2*inc-1)])
    distances_to_obst['front'] = min(laser_readings['ranges'][30+(2*inc):30+(3*inc-1)])
    distances_to_obst['front_left'] = min(laser_readings['ranges'][30+(3*inc):30+(4*inc-1)])
    distances_to_obst['left'] = min(laser_readings['ranges'][30+(4*inc):30+(5*inc-1)])
    distances_to_obst['back_left'] = min(laser_readings['ranges'][30+5*inc:239])

    if (min(laser_readings['ranges']) < laser_readings['range_max']):
        laser_readings['min_dist_index'] = min(enumerate(laser_readings['ranges']), key=itemgetter(1))[0]
    else:
        laser_readings['min_dist_index'] = -1

    #rospy.loginfo("Index of the closest ray to wall: " + str(laser_readings['min_dist_index']))


def check_outer_corner():
    global laser_readings

    max_distance = laser_readings['range_max']

    if (
        (
            distances_to_obst['back_left'] < max_distance
            and distances_to_obst['left'] > max_distance
            and distances_to_obst['front_left'] > max_distance
            and distances_to_obst['front'] > max_distance
            and distances_to_obst['front_right'] > max_distance
            and distances_to_obst['right'] > max_distance
            and distances_to_obst['back_right'] > max_distance
        )
        or
        (
            distances_to_obst['back_right'] < max_distance
            and distances_to_obst['left'] > max_distance
            and distances_to_obst['front_left'] > max_distance
            and distances_to_obst['front'] > max_distance
            and distances_to_obst['front_right'] > max_distance
            and distances_to_obst['right'] > max_distance
            and distances_to_obst['back_right'] > max_distance
        )
    ):
        return True

    return False

def check_inner_corner():
    global laser_readings

    max_distance = laser_readings['range_max']

    if (
        distances_to_obst['front'] < max_distance
        and distances_to_obst['front_right'] < max_distance
        and distances_to_obst['front_left'] < max_distance
    ):
        return True

    return False

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
        current_state = state['GO']
    msg_to_send = Twist()
    msg_to_send.linear.x = SPEED
    return msg_to_send

def go():
    global distances_to_obst, laser_readings, current_state, k, alpha

    num_rays = len(laser_readings['ranges'])
    max_distance = laser_readings['range_max']
    msg_to_send = None

    if (check_outer_corner()): # Outer corner detected, turn into it and continue going
        rospy.loginfo("Outer corner detected")
        msg_to_send = Twist()
        msg_to_send.linear.x = SPEED/2
        if (distances_to_obst['back_left'] < max_distance):
            msg_to_send.angular.z = ANGULAR_SPEED
        else:
            msg_to_send.angular.z = -ANGULAR_SPEED
    if (check_inner_corner()): # Inner corner detected, turn away from it
        rospy.loginfo("Inner corner detected")
        msg_to_send = Twist()
        if (distances_to_obst['right'] < max_distance):
            msg_to_send.angular.z = ANGULAR_SPEED
        else:
            msg_to_send.angular.z = -ANGULAR_SPEED
    elif ( # No obstacle in front, just follow the wall and correct yourself
        distances_to_obst['left'] < max_distance
        or distances_to_obst['front_left'] < max_distance
        or distances_to_obst['front_right'] < max_distance
        or distances_to_obst['right'] < max_distance
    ):
        rospy.loginfo("Following wall")
        if (laser_readings['min_dist_index'] is not -1):
            if (laser_readings['min_dist_index'] <= math.floor(num_rays/2)):
                alpha = math.floor(num_rays/2) - laser_readings['angle_increment'] * laser_readings['min_dist_index'] 
                alpha = (math.pi/180) * alpha
                msg_to_send = Twist()
                msg_to_send.linear.x = SPEED/2
                msg_to_send.angular.z = k * alpha
            else:
                alpha = laser_readings['angle_increment'] * laser_readings['min_dist_index'] - math.floor(num_rays/2)
                alpha = (math.pi/180) * alpha
                msg_to_send = Twist()
                msg_to_send.linear.x = SPEED/2
                msg_to_send.angular.z = -(k * alpha)
        else:
            current_state = state['FIND']

    return msg_to_send

def follow():
    return None

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
    global SPEED, ANGULAR_SPEED, current_state, laser_readings

    rospy.init_node('wall_following', anonymous=True)
    rospy.Subscriber(robot_frame_id + '/' + laser_frame_id, LaserScan, laser_callback)
    publisher = rospy.Publisher(robot_frame_id + '/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg_to_send = None

        if current_state == state['FIND']:
            msg_to_send = find()
        elif current_state == state['GO']:
            msg_to_send = go()
        elif current_state == state['FOLLOW']:
            msg_to_send = follow()
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