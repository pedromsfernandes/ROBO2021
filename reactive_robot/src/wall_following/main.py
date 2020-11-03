#!/usr/bin/env python

# IMPORTANT! You need to add execute permissions to the file (chmod +x main.py)

import sys
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Parameters in sensor_msgs LaserScan (NOTE: all floats unless otherwise stated)
# range_min, range_max, min, max, dahead, dleft, dright, ranges([]), angle_min, angle_max, angle_increment, time_increment, scan_time, intensities([])

# NOTE: Twist - linear and angular velocity (both Vector3)

SPEED = 0.2
ANGULAR_SPEED = 0.4

def callback(data):
    pass

def wall_following(robot_frame_id, laser_frame_id):
    global SPEED, ANGULAR_SPEED
    
    rospy.init_node('wall_following', anonymous=True)
    rospy.Subscriber(robot_frame_id + '/' + laser_frame_id, LaserScan, callback)
    publisher = rospy.Publisher(robot_frame_id + '/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg_to_send = Twist()
        msg_to_send.linear.x = SPEED
        msg_to_send.angular.z = ANGULAR_SPEED
        publisher.publish(msg_to_send)
        rate.sleep()

if __name__ == '__main__':
    try:
        wall_following(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass