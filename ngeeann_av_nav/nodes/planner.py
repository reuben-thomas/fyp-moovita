#!/usr/bin/env python

import rospy, os, cubic_spline_planner
import numpy as np
import pandas as pd

from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from nav_msgs.msg import Path
from ngeeann_av_nav.msg import Path2D

class PathPlanner:

    def __init__(self):

        # Initialise publishers
        self.path_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=30)
        # self.path_viz_pub = rospy.Publisher('/nggeeann_av/viz_path', Path, queue_size=30) 

        # Load parameters
        self.planner_params = rospy.get_param("/path_planner")
        self.frequency = self.planner_params["update_frequency"]
        self.frame_id = self.planner_params["frame_id"]

        # Class constants
        self.halfpi = np.pi / 2.0
        self.ds = 0.1

        # Class variables to use whenever within the class when necessary
        self.ax = [101.835, 100.0, 100.0, 96.0, 90.0]
        self.ay = [10, 18.3, 31.0, 43.0, 47.0]

    def create_pub_path(self):

        cx, cy, cyaw, _, _ = cubic_spline_planner.calc_spline_course(self.ax, self.ay, self.ds)
        target_path = Path2D()

        for n in range(0, len(cx)):
            npose = Pose2D()
            npose.x  = cx[n]
            npose.y = cy[n]

            # Aligns target heading to y-axis
            npose.theta = -cyaw[n] + self.halfpi
            if (npose.theta < 0.0):
                npose.theta = (2.0 * np.pi) + npose.theta

            target_path.poses.append(npose)

        rospy.loginfo("Total Points: {}".format(len(target_path.poses)))

        self.path_planner_pub.publish(target_path)

    # Consecutively constructs and publishes path in Path2D and Path message, visualized in rviz (Requires map frame)
    def create_viz_path(self):
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

        self.path_planner_pub.publish(target_path)

    # Converts yaw heading to quaternion
    def heading_to_quaternion(self, heading):

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(heading / 2.0)
        quaternion.w = np.cos(heading / 2.0)

        return quaternion

    def walk_up_folder(self, path, dir_goal='fyp-moovita'):

        dir_path = os.path.dirname(path)
        split_path = str.split(dir_path, '/')     
        counter = 0  

        while (split_path[-1] != dir_goal and counter < 20.0):
            dir_path = os.path.dirname(dir_path)
            split_path = str.split(dir_path, '/')
            counter += 1.0
        
        return dir_path

def main():

    # Initialise the class
    path_planner = PathPlanner()

    # Initialise the node
    rospy.init_node('path_planner')

    # Set update rate
    r = rospy.Rate(path_planner.frequency) 


    while not rospy.is_shutdown():
        try:

            # Create path
            path_planner.create_pub_path()
            r.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ROS node...")

if __name__=="__main__":
    main()