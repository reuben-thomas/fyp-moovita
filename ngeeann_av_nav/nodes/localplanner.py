#!/usr/bin/env python

import rospy, os
import numpy as np

from utils.cubic_spline_planner import *
from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import Path

class LocalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publishers
        self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)
        self.path_viz_pub = rospy.Publisher('/nggeeann_av/viz_path', Path, queue_size=10)

        # Initialise subscribers
        self.goals_sub = rospy.Subscriber('/ngeeann_av/goals', Path2D, self.goals_cb, queue_size=10)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)

        # Load parameters
        try:
            self.planner_params = rospy.get_param("/local_path_planner")
            self.frequency = self.planner_params["update_frequency"]
            self.frame_id = self.planner_params["frame_id"]

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.halfpi = np.pi / 2
        self.ds = 0.1

        # Class variables to use whenever within the class when necessary
        self.ax = []
        self.ay = []

    def goals_cb(self, msg):

        ''' Callback function to receive waypoint data from the Global Path Planner '''

        self.ax = []
        self.ay = []
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)

        print("\nGoals received: {}".format(len(msg.poses)))

    def vehicle_state_cb(self, msg):

        ''' Callback function to receive information on the vehicle's vertical and horizontal coordinates '''

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def create_pub_path(self):

        ''' Uses the cubic_spline_planner library to interpolate a cubic spline path over the given waypoints '''

        cx, cy, cyaw, a, b = calc_spline_course(self.ax, self.ay, self.ds)
        
        cells = min(len(cx), len(cy), len(cyaw))

        target_path = Path2D()
        
        viz_path = Path()
        viz_path.header.frame_id = "map"
        viz_path.header.stamp = rospy.Time.now()

        for n in range(0, cells):
            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = self.frame_id
            vpose.header.seq = n
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = self.heading_to_quaternion(np.pi * 0.5 - cyaw[n])
            viz_path.poses.append(vpose)

        self.local_planner_pub.publish(target_path)
        self.path_viz_pub.publish(viz_path)

    def heading_to_quaternion(self, heading):

        ''' Converts yaw heading to quaternion coordinates '''

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(heading / 2)
        quaternion.w = np.cos(heading / 2)

        return quaternion

def main():

    ''' Main function to initialise the class and node. '''

    # Initialise the class
    local_planner = LocalPathPlanner()

    # Initialise the node
    rospy.init_node('local_planner')

    # Set update rate
    r = rospy.Rate(local_planner.frequency) 

    # Wait for messages
    rospy.wait_for_message('/ngeeann_av/goals', Path2D)

    while not rospy.is_shutdown():
        try:
            local_planner.create_pub_path()

            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()