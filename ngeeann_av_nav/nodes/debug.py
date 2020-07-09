#!/usr/bin/env python

import rospy, cubic_spline_planner
import numpy as np

from geometry_msgs.msg import Pose2D
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path

class PathTracker:

    def __init__(self):

        # Initialise publishers and subscribers
        self.tracker_pub = rospy.Publisher('/ngeeann_av/ackermann_cmd', AckermannDrive, queue_size=30)
        
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', Pose2D, self.vehicle_state_cb, queue_size=30)
        self.path_sub = rospy.Subscriber('/ngeeann_av/path', Path, self.path_cb, queue_size=30)

        # Load parameters (Future)
        # self.tracker_params = rospy.get_param("/path_tracker")
        # self.target_vel = self.tracker_params["target_velocity"]
        # self.k = self.tracker_params["control_gain"]
        # self.ksoft = self.tracker_params["softening_gain"]
        # self.max_steer = self.tracker_params["steering_limits"]
        # self.cog2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]

        # Class constants
        self.target_vel = 5.0
        self.k = 1.0
        self.ksoft = 1.0
        self.max_steer = 0.95
        self.cog2frontaxle = 1.483
        self.halfpi = np.pi / 2

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.yaw = None

        self.cx = []
        self.cy = []
        self.cyaw = []

        self.dy = []
        self.dx = []

    def vehicle_state_cb(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta

    def path_cb(self, msg):
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].pose.position.x
            py = msg.poses[i].pose.position.y
            orientation = 2.0 * np.arctan2(msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.w)
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(orientation)

def main():

    # Initialise the class
    path_tracker = PathTracker()

    # Initialise the node
    rospy.init_node('path_tracker')

    # Set update rate, default to 30
    r = rospy.Rate(30)

    try:
        rospy.loginfo("x:{}".format(path_tracker.x)) 
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down ROS node...")

if __name__ == "__main__":
    main()