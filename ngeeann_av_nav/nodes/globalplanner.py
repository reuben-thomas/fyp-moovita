#!/usr/bin/env python

import rospy, os
import numpy as np
import pandas as pd

from geometry_msgs.msg import Pose2D, Quaternion, PoseStamped
from nav_msgs.msg import Path
from ngeeann_av_nav.msg import Path2D, State2D
from std_msgs.msg import String

class GlobalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publisher(s)
        self.goals_pub = rospy.Publisher('/ngeeann_av/goals', Path2D, queue_size=10)
        self.goals_viz_pub = rospy.Publisher('/ngeeann_av/viz_goals', Path, queue_size=10)
        self.success_pub = rospy.Publisher('/ngeeann_av/success', String, queue_size=10)
        self.initialised_pub = rospy.Publisher('/ngeeann_av/globalplanner_hb', String, queue_size=10)

        # Initialise suscriber(s)
        self.initialised_sub = rospy.Subscriber('/ngeeann_av/localplanner_hb', String, self.initialised_cb, queue_size=10)
        # self.targets_sub = rospy.Subscriber('/ngeeann_av/current_target', Pose2D, self.target_check_cb, queue_size=10)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)

        # Load parameters
        try:
            self.global_planner_params = rospy.get_param("/global_path_planner")
            self.frequency = self.global_planner_params["update_frequency"]
            self.givenwp = self.global_planner_params["given_number_of_waypoints"]
            self.tolerance = self.global_planner_params["target_tolerance"]

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
        # self.ay[1] = 47

        # Class variables to use whenever within the class when necessary
        self.alive = False

        self.x = None
        self.y = None

        self.lowerbound = 0
        self.upperbound = self.lowerbound + (self.givenwp)

        self.lowerindex = 0
        self.upperindex = self.lowerindex + (self.givenwp - 1)

        self.ax_pub = self.ax[self.lowerbound : self.upperbound]
        self.ay_pub = self.ay[self.lowerbound : self.upperbound]

        self.current_target = None

        self.points = 1
        self.total_goals = 0

        self.target_x = None
        self.target_y = None

    def initialised_cb(self, msg):

        ''' Callback function to check if the Local Path Planner has been initialised '''

        if msg.data == "I am alive!":
            self.alive = True
        
        else:
            self.alive = False

    def vehicle_state_cb(self, msg):
        
        self.x = msg.pose.x
        self.y = msg.pose.y

        # if np.around(self.x) == np.around(self.ax[self.lowerindex + 1]) and np.around(self.y) == np.around(self.ay[self.lowerindex + 1]):
        #     self.reached(True)
            
        # else:
        #     pass

    # def target_check_cb(self, msg):

    #     ''' Callback function to receive information on the vehicle's current target '''

    #     self.target_x = msg.x
    #     self.target_y = msg.y

    #     if np.around(msg.x) == np.around(self.ax[self.lowerindex + 1]) and np.around(msg.y) == np.around(self.ay[self.lowerindex + 1]):
    #         self.reached(True)
            
    #     else:
    #         pass

    def reached(self, reached):

        ''' Tells the node when to compute and publish the waypoints to the Local Path Planner '''
        
        # If the vehicle has reached the goal
        if reached == True:
            self.set_waypoints(False)
            self.success_pub.publish("Reached.")

            self.points += 1

        else:
            print("Not yet reached.")
            pass

    def set_waypoints(self, first):

        ''' Set the waypoints that are to be published, given the vehicle's position ''' 

        if first == True:

            # If there is not enough waypoints to publish
            if  self.givenwp >= len(self.ax):
                
                # Publish entire array
                self.publish_goals(self.ax, self.ay)

            else:
                self.publish_goals(self.ax_pub, self.ay_pub)

        else:
            if  self.givenwp > (len(self.ax) - self.lowerindex): # If the number of waypoints to give is more than the number of waypoints left
                self.upperbound = self.lowerbound + (len(self.ax) - self.lowerbound) # New upper index

                self.ax_pub = self.ax[self.lowerbound - 1 : self.upperbound]
                self.ay_pub = self.ay[self.lowerbound - 1 : self.upperbound]

                self.success_pub.publish("Success.")

            else: # If the number of waypoints to give is less or equal to the number of waypoints left.
                self.lowerbound += 1
                self.upperbound += 1
                self.lowerindex += 1
                self.upperindex += 1
                
                self.ax_pub = self.ax[self.lowerbound - 1 : self.upperbound]
                self.ay_pub = self.ay[self.lowerbound - 1 : self.upperbound]

            print("ax_pub:{}\nay_pub{}".format(self.ax_pub, self.ay_pub))
            self.publish_goals(self.ax_pub, self.ay_pub)

    def publish_goals(self, ax, ay):

        ''' Publishes an array of waypoints for the Local Path Planner '''
        goals = Path2D()

        viz_goals = Path()
        viz_goals.header.frame_id = "map"
        viz_goals.header.stamp = rospy.Time.now()

        goals_per_publish = 0
        self.total_goals += 1

        for i in range(0, self.givenwp):
            # Appending to Target Goals
            goal = Pose2D()
            goal.x = ax[i]
            goal.y = ay[i]
            goals.poses.append(goal)
            
            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = "map"
            vpose.header.seq = i
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = ax[i]
            vpose.pose.position.y = ay[i]
            vpose.pose.position.z = 0.0
            viz_goals.poses.append(vpose)
            
            goals_per_publish += 1

        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)

        print("\n")
        print("Total goals published: {}".format(self.total_goals))
        print("Goals per publish:{}".format(goals_per_publish))
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

    ''' Main function to initialise the class and node. '''
    
    # Initialise the class
    global_planner = GlobalPathPlanner()

    # Initialise the node
    rospy.init_node('global_planner')

    # Set update rate
    r = rospy.Rate(global_planner.frequency)

    while not rospy.is_shutdown():
        if global_planner.alive == True:
            global_planner.initialised_pub.publish("I am alive!")
            print("\nLocal planner is awake.")
            break

        else:
            r.sleep()

    # Publishes the first goal
    global_planner.set_waypoints(True)

    while not rospy.is_shutdown():
        try:
            print("\nVehicle Position: ({}, {})".format(np.around(global_planner.x), np.around(global_planner.y)))
            print("Goal: ({}, {})".format(np.around(global_planner.ax[global_planner.lowerindex + 1]), np.around(global_planner.ay[global_planner.lowerindex + 1])))

            if np.around(global_planner.x) == np.around(global_planner.ax[global_planner.lowerindex + 1]) and np.around(global_planner.y) == np.around(global_planner.ay[global_planner.lowerindex + 1]):
                global_planner.reached(True)

            else:
                pass

            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()