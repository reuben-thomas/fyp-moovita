#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Pose2D
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path

class BOF:
    
    def __init__(self):

def main():

    # Initialise the class
    bof = BOF()

    # Initialise the node
    rospy.init_node('bof')

    # Set update rate
    r = rospy.Rate(path_tracker.frequency)

    while not rospy.is_shutdown():
        try:
            r.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ROS node...")

if __name__ == "__main__":
    main()