#!/usr/bin/env python

# IMPORTANT! You need to add execute permissions to the file (chmod +x main.py)

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# Parameters in sensor_msgs LaserScan (NOTE: all floats unless otherwise stated)
# range_min, range_max, min, max, dahead, dleft, dright, ranges([]), angle_min, angle_max, angle_increment, time_increment, scan_time, intensities([])

def callback(data):
    rospy.loginfo("Got a message")

def wall_following(robot_frame_id, laser_frame_id):
    rospy.init_node('wall_following', anonymous=True)
    rospy.Subscriber(robot_frame_id + '/' + laser_frame_id, LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        wall_following(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass