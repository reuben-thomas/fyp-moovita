#!/usr/bin/env python

import rospy, os, cubic_spline_planner
import numpy as np
import pandas as pd

from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from nav_msgs.msg import Path
from ngeeann_av_nav.msg import Path2D
from std_msgs.msg import String

class LocalPathPlanner:

    def __init__(self):

        # Initialise publishers
        self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)
        # self.path_viz_pub = rospy.Publisher('/nggeeann_av/viz_path', Path, queue_size=10)
        self.initialised_pub = rospy.Publisher('/ngeeann_av/localplanner_hb', String, queue_size=10)

        # Initialise subscribers
        self.goals_sub = rospy.Subscriber('/ngeeann_av/goals', Path2D, self.goals_cb, queue_size=10)
        self.initialised_sub = rospy.Subscriber('/ngeeann_av/globalplanner_hb', String, self.initilialised_cb, queue_size=10)

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
        self.alive = False

    def initilialised_cb(self, msg):
        
        if msg.data == "I am alive!":
            self.alive = True
        
        else:
            self.alive = False

    def goals_cb(self, msg):
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)

    def create_pub_path(self):

        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(self.ax, self.ay, self.ds)
        target_path = Path2D()

        for n in range(0, len(cx)):
            npose = Pose2D()
            npose.x  = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

        rospy.loginfo("Total Points: {}".format(len(target_path.poses)))

        self.local_planner_pub.publish(target_path)

    def create_viz_path(self):

        ''' Consecutively constructs and publishes path in Path2D and Path message, visualized in rviz (Requires map frame) '''

        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(self.ax, self.ay, self.ds)
        target_path = Path()
        target_path.header.frame_id = self.frame_id
        target_path.header.stamp = rospy.Time.now()
        target_path.header.seq = count

        for n in range(0, len(cx)):
            npose = PoseStamped()
            npose.header.frame_id = self.frame_id
            npose.header.seq = n
            npose.header.stamp = rospy.Time.now()
            npose.pose.position.x = cx[n]
            npose.pose.position.y = cy[n]
            npose.pose.position.z = 0.0
            npose.pose.orientation = self.heading_to_quaternion(cyaw[n])
            target_path.poses.append(npose)

        rospy.loginfo("Total Points: {}".format(len(target_path.poses)))

        self.local_planner_pub.publish(target_path)

    def heading_to_quaternion(self, heading):

        ''' Converts yaw heading to quaternion'''

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(heading / 2)
        quaternion.w = np.cos(heading / 2)

        return quaternion

def main():

    # Initialise the class
    local_planner = LocalPathPlanner()

    # Initialise the node
    rospy.init_node('local_planner')

    # Set update rate
    r = rospy.Rate(local_planner.frequency) 

    while not rospy.is_shutdown():
        try:
            local_planner.initialised_pub.publish("I am alive!")

            if local_planner.alive == True:
                # Create path
                local_planner.create_pub_path()
                r.sleep()

            else:
                r.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ROS node...")

if __name__=="__main__":
    main()