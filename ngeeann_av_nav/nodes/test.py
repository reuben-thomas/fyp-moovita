#!/usr/bin/env python

import rospy, cubic_spline_planner
import numpy as np

from geometry_msgs.msg import Pose2D
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path

class PathTracker:

    def __init__(self):

        # Initialise subscribers
        self.path = rospy.Subscriber('/ngeeann_av/path', Path, self.path_cb, queue_size=30)
        self.localisation = rospy.Subscriber('/ngeeann_av/state2D', Pose2D, self.vehicle_state_cb, queue_size=30)

        # self.tracker_params = rospy.get_param("/path_tracker")
        # self.target_vel = self.tracker_params("target_velocity")
        # self.k = self.tracker_params["control_gain"]
        # self.ksoft = self.tracker_params["softening_gain"]
        # self.max_steer = self.tracker_params["steering_limits"]
        # self.cog2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]

        self.halfpi = np.pi / 2

        self.x = None
        self.y = None
        self.theta = None

        self.cx = None
        self.cy = None
        self.cyaw = None

        self.test = None

    def vehicle_state_cb(self, data):

        self.x = data.x
        self.y = data.y
        self.theta = data.theta
        # rospy.loginfo("x: {}, y: {}, theta: {}".format(self.x, self.y, self.theta))

    def path_cb(self, data):
        self.cx = data.poses.pose.position.x
        self.cy = data.poses.pose.position.y
        self.cyaw = data.poses.pose.orientation
        rospy.loginfo("x: {}, y: {}, cyaw: {}".format(self.cx, self.cy, self.cyaw))

def main():

    # Initialise the class
    path_tracker = PathTracker()

    # Initialise the node
    rospy.init_node('path_tracker')

    # Set update rate, default to 30
    r = rospy.Rate(30)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down ROS node")

if __name__=="__main__":
    main()