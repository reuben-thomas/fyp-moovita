#!/usr/bin/env python

import rospy, os
import numpy as np
import pandas as pd
import PyKDL

from geometry_msgs.msg import Pose2D, Quaternion, Pose, PoseArray
from nav_msgs.msg import Path
from ngeeann_av_nav.msg import Path2D, State2D
from std_msgs.msg import String

class GlobalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publisher(s)
        self.goals_pub = rospy.Publisher('/ngeeann_av/goals', Path2D, queue_size=10)
        self.goals_viz_pub = rospy.Publisher('/ngeeann_av/viz_goals', PoseArray, queue_size=10)
        self.initialised_pub = rospy.Publisher('/ngeeann_av/globalplanner_hb', String, queue_size=10)

        # Initialise suscriber(s)
        self.initialised_sub = rospy.Subscriber('/ngeeann_av/localplanner_hb', String, self.initialised_cb, queue_size=10)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)

        # Load parameters
        try:
            self.global_planner_params = rospy.get_param("/global_path_planner")
            self.frequency = self.global_planner_params["update_frequency"]
            self.givenwp = self.global_planner_params["given_number_of_waypoints"]
            self.tolerance = self.global_planner_params["target_tolerance"]
            self.tracker_params = rospy.get_param("/path_tracker")
            self.cg2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]

            if self.givenwp < 2:
                self.givenwp == 2
            else:
                pass
        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Get path to waypoints.csv
        dir_path = os.path.dirname(os.path.abspath(__file__))
        dir_path = self.walk_up_folder(dir_path)
        df = pd.read_csv(os.path.join(dir_path, 'scripts', 'waypoints.csv'))

        print("Waypoint directory: {}".format(os.path.join(dir_path, 'scripts', 'waypoints.csv')))

        # Import waypoints.csv into class variables ax and ay
        self.ax = df['X-axis'].values.tolist()
        self.ay = df['Y-axis'].values.tolist()

        # Class variables to use whenever within the class when necessary
        self.alive = False

        self.x = None
        self.y = None
        self.theta = None

    def initialised_cb(self, msg):
        ''' Callback function to check if the Local Path Planner has been initialised '''

        if msg.data == "I am alive!":
            self.alive = True
        
        else:
            self.alive = False

    def vehicle_state_cb(self, msg):
        ''' Callback function to update vehicle state '''

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta
        self.find_closest_point()

    def find_closest_point(self):
        # Identify position of vehicle front axle
        fx = self.x + self.cg2frontaxle * np.cos(self.theta)
        fy = self.y + self.cg2frontaxle * np.sin(self.theta)

        dx = [fx - icx for icx in self.ax] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.ay] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        closest_id = np.argmin(d) # Find the shortest distance in the array

        self.frame_transform(closest_id, fx, fy)

    def frame_transform(self, closest_id, axle_x, axle_y):
        ''' Recieves position of vehicle front axle, and id of closest waypoint. This waypoint is transformed from
            "map" frame to the vehicle frame

            Arguments:
                closest_id      - Index to closest waypoint to front axle in master waypoint list
                axle_x, axle_y  - Coordinates (x,y) of vehicle front axle position
        '''
        # Creates vehicle frame
        vehicle_frame = PyKDL.Frame(PyKDL.Rotation.RPY(self.theta, 0, 0), PyKDL.Vector(-self.x, -self.y, 0.0))

        # Position vector of closest waypoint in map frame
        p_map = PyKDL.Vector(self.ax[closest_id], self.ay[closest_id], 0.0)

        # Transformation of waypoint to vehicle frame
        p_vehicle = vehicle_frame * p_map
        print('Waypoint #: {}'.format(closest_id))
        print('Map frame: {}'.format(p_map))
        print('Vehicle_frame: {}'.format(p_vehicle))



    def publish_goals(self, ax, ay):
        ''' Publishes an array of waypoints for the Local Path Planner '''

        goals = Path2D()

        viz_goals = PoseArray()
        viz_goals.header.frame_id = "map"
        viz_goals.header.stamp = rospy.Time.now()

        for i in range(0, len(ax)):
            # Appending to Target Goals
            goal = Pose2D()
            goal.x = ax[i]
            goal.y = ay[i]
            goals.poses.append(goal)
            
            # Appending to Visualization Path
            vpose = Pose()
            vpose.position.x = ax[i]
            vpose.position.y = ay[i]
            vpose.position.z = 0.0
            viz_goals.poses.append(vpose)
        
        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)

        print("\n")
        print("Total goals published: {}".format(len(ax)))
        print("\nPublished goals: \n{}".format(goals))

    def walk_up_folder(self, path, dir_goal='fyp-moovita'):
        ''' Searches and returns the directory of the waypoint.csv file ''' 

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

    while not rospy.is_shutdown():
        try:
            global_planner.publish_goals(global_planner.ax, global_planner.ay)
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()