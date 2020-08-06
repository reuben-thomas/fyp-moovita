#!/usr/bin/env python

import rospy, os
import numpy as np
import pandas as pd
import PyKDL
import threading

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
            self.tracker_params = rospy.get_param("/path_tracker")
            self.cg2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]
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

        self.lock = threading.Lock()

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

    def set_waypoints(self):
        ''' Determines the appropriate set of waypoints to publish by the following steps

            1. Identify waypoint closest to front axle
            2. Determines if this point is ahead or behind, by transformation
            3. Preserves at least 1 point behind, 5 points ahead at all times
        '''
        # Identify position of vehicle front axle
        fx = self.x + self.cg2frontaxle * np.cos(self.theta + np.pi*0.5)
        fy = self.y + self.cg2frontaxle * np.sin(self.theta + np.pi*0.5)

        dx = [fx - icx for icx in self.ax] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.ay] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy)        # Find the distance from the front axle to the path
        closest_id = np.argmin(d)   # Find the shortest distance in the array

        transform = self.frame_transform(self.ax[closest_id], self.ay[closest_id], fx, fy, self.theta)
        
        if (transform[1] <= 0.0):
        # If the vehicle has passed, closest point is preserved as a point behind the car
            px = self.ax[closest_id - 1 : (closest_id + 4)]
            py = self.ay[closest_id - 1: (closest_id + 4)]
            print('Closest Waypoint #: {} (Passed)'.format(closest_id))

        else:
        # If the vehicle has yet to pass, a point behind the closest is preserved as a point behind the car
            px = self.ax[(closest_id - 2) : (closest_id + 3)]
            py = self.ay[(closest_id - 2) : (closest_id + 3)]
            print('Closest Waypoint #: {} (Approaching)'.format(closest_id))
        
        self.publish_goals(px, py)

    def frame_transform(self, point_x, point_y, axle_x, axle_y, theta):
        ''' Recieves position of vehicle front axle, and id of closest waypoint. This waypoint is transformed from
            "map" frame to the vehicle frame

            Arguments:
                closest_id          - Index to closest waypoint to front axle in master waypoint list
                point_x, point_y    - Coordinates (x,y) of vehicle front axle position
                axle_x, axle_y      - Coordinates (x,y) of vehicle front axle position
        '''
        c, s = np.cos(-theta), np.sin(-theta)     # Creates rotation matrix given theta
        R = np.array(((c, -s), (s, c)))  

        p = np.array(((point_x), (point_y)))      # Position vector of closest waypoint (world frame)
        v = np.array(((axle_x), (axle_y)))        # Position vector of vehicle (world frame)
        vp = p - v                                # Linear translation between vehicle and point     

        transform = R.dot(vp)      # Product of rotation matrix and translation vector
        return transform

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
            vpose.orientation
            viz_goals.poses.append(vpose)
        
        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)

        print("\n")
        print("Total goals published: {}".format(len(ax)))
        #print("\nPublished goals: \n{}".format(goals))

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

    rospy.wait_for_message('/ngeeann_av/state2D', State2D)

    # Set update rate
    r = rospy.Rate(global_planner.frequency)

    while not rospy.is_shutdown():
        try:
            global_planner.set_waypoints()
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()