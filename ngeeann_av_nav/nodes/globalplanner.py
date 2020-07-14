#!/usr/bin/env python

import rospy, os
import numpy as np
import pandas as pd

from geometry_msgs.msg import Pose2D
from ngeeann_av_nav.msg import Path2D, State2D

class GlobalPathPlanner:

    def __init__(self):

        # Initialise publisher(s)
        self.goals_pub = rospy.Publisher('/ngeeann_av/goals', Path2D, queue_size=10)

        # Initialise suscriber(s)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=50)

        # Load parameters
        try:
            self.global_planner_params = rospy.get_param("/global_path_planner")
            self.frequency = self.global_planner_params["update_frequency"]
            self.givenwp = self.global_planner_params["number_of_waypoints"]

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Get path to waypoints.csv
        dir_path = os.path.dirname(os.path.abspath(__file__))
        dir_path = self.walk_up_folder(dir_path)
        df = pd.read_csv(os.path.join(dir_path, 'scripts', 'waypoints.csv'))

        # Import waypoints.csv into class variables ax and ay
        self.ax = df['X-axis'].values.tolist()
        self.ay = df['Y-axis'].values.tolist()
        # self.ay[1] = 47

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.lowerbound = 0
        self.upperbound = self.lowerbound + (self.givenwp)
        self.ax_pub = self.ax[0 : self.upperbound]
        self.ay_pub = self.ay[0 : self.upperbound]

    def vehicle_state_cb(self, msg):

        self.x = msg.pose.x
        self.y = msg.pose.y

    def almost_reached(self):
        
        # If the vehicle has almost reached the goal
        if self.x == self.ax[self.upperbound - 1] and self.y == self.ay[self.upperbound - 1]:
            self.set_waypoints(False)
        
        else:
            pass

    def set_waypoints(self, first):

        if first == True:
            self.publish_goals(self.ax_pub, self.ay_pub)

        else:
            # If the number of waypoints to give is less or equal than the number of waypoints left
            if (len(self.ax) - self.upperbound) <= self.givenwp:
                self.ax_pub = self.ax[self.lowerbound : self.upperbound]
                self.ay_pub = self.ay[self.lowerbound : self.upperbound]

            else:
                # If the number of waypoints to give is more than the number of waypoints left.
                self.upperbound = len(self.ax)
                self.ax_pub = self.ax[self.lowerbound : self.upperbound]
                self.ay_pub = self.ay[self.lowerbound : self.upperbound]

            self.publish_goals(self.ax_pub, self.ay_pub)
        
        self.lowerbound += self.givenwp
        self.upperbound += self.givenwp

    def publish_goals(self, ax, ay):

        goals = Path2D()

        count = 0

        for i in range(0, self.givenwp):
            goal = Pose2D()
            goal.x = ax[i]
            goal.y = ay[i]
            goal.theta = None
            print ax[i]
            
            goals.poses.append(goal)
            count += 1

        self.goals_pub.publish(goals)

        print("Published Waypoints:{}\n{}".format(count, goals))

    def walk_up_folder(self, path, dir_goal='fyp-moovita'):

        dir_path = os.path.dirname(path)
        split_path = str.split(dir_path, '/')     
        counter = 0  

        while (split_path[-1] != dir_goal and counter < 20):
            dir_path = os.path.dirname(dir_path)
            split_path = str.split(dir_path, '/')
            counter += 1
    
        return dir_path
        
def main():
    
    # Initialise the class
    global_planner = GlobalPathPlanner()

    # Initialise the node
    rospy.init_node('global_planner')

    # Set update rate
    r = rospy.Rate(global_planner.frequency)

    # Publishes the first goal
    global_planner.set_waypoints(True)

    while not rospy.is_shutdown():
        try:
            global_planner.almost_reached()
            r.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ROS node...")

if __name__=="__main__":
    main()