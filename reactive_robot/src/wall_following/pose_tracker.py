#! /usr/bin/env python

import rospy
import time
import tf
from datetime import timedelta
from datetime import datetime

if __name__ == "__main__":
    rospy.init_node('wall_following', anonymous=True)
    listener = tf.TransformListener()
    with open('pose_tracker.csv', 'w') as file:
        file.write("time, x, y\n")
        start = datetime.now()
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            curr = datetime.now()
            delta = curr - start
            seconds = delta.total_seconds()

            try:
                (trans,rot) = listener.lookupTransform('map_static', "robot0", rospy.Time(0))
                file.write(str(seconds) + ", " + str(trans[0]) + ", " + str(trans[1]) + "\n")
                print(str(seconds) + ", " + str(trans[0]) + ", " + str(trans[1]))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
            
