#!/usr/bin/env python

import rospy, os
import numpy as np
import pandas as pd

from ngeeann_av_nav.msg import Path2D

class GlobalPlanner:

    def __init__(self):

        # Initialise publisher(s)
        self.path_planner_pub = rospy.Publisher('/ngeeann_av/goals', Path2D, queue_size=30)

        # Initialise suscriber(s)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=50)

        # Load parameters
        try:
            self.global_planner_params = rospy.get_param("/path_planner")
            self.frequency = self.global_planner_params["update_frequency"]
            
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
    global_planner = GlobalPlanner()

    # Initialise the node
    rospy.init_node('global_planner')

    # Set update rate
    r = rospy.Rate(global_planner.frequency)

if __name__=="__main__":
    main()