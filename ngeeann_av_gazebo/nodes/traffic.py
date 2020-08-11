#!/usr/bin/env python
import rospy, os
import numpy as np
import pandas as pd

from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from ngeeann_av_nav.msg import State2D

class Traffic:

    def __init__(self):

        # Wait and initialise service
        rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state')
        rospy.wait_for_service('/ngeeann_av/gazebo/set_model_state')
        self.get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)
        self.set_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/set_model_state', SetModelState)

        # Load parameters
        try:
            self.localisation_params = rospy.get_param("/localisation")
            self.frequency = self.localisation_params["update_frequency"]
            self.model = self.localisation_params["model_name"]

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state = None

        # Get path to waypoints.csv
        dir_path = os.path.dirname(os.path.abspath(__file__))
        dir_path = self.walk_up_folder(dir_path)
        df = pd.read_csv(os.path.join(dir_path, 'scripts', 'traffic.csv'))
        print("Waypoint directory: {}".format(os.path.join(dir_path, 'scripts', 'traffic.csv')))
        self.ax = df['X-axis'].values.tolist()
        self.ay = df['Y-axis'].values.tolist()

    # Gets vehicle position from Gazebo and publishes data
    def recieve_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.position.x
        state2d.pose.y = self.state.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.orientation.z, self.state.pose.orientation.w)
        
        # Aligning heading to y-axis, accounts for double rotation error
        if state2d.pose.theta < 0.0:
            state2d.pose.theta += 2.0 * np.pi
        
        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.linear.x
        state2d.twist.y = self.state.twist.linear.y
        state2d.twist.w = -self.state.twist.angular.z

        # Print state
        print("Position (x,y): ({},{})".format(round(state2d.pose.x, 5), round(state2d.pose.y, 5)))
        print("Heading: {}".format(round(state2d.pose.theta, 5)))
        print("Velocity (x,y): ({},{})".format(round(state2d.twist.x, 5), round(state2d.twist.y, 5)))

    def set_state(self, id):
        state_msg = ModelState()
        state_msg.model_name = 'suv'
        state_msg.pose.position.x = self.ax[id]
        state_msg.pose.position.y = self.ay[id]
        state_msg.pose.position.z = 0.0
        
        heading = np.arctan2(state_msg.pose.position.y, state_msg.pose.position.x) + np.pi * 0.5

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = np.sin(heading / 2)
        state_msg.pose.orientation.w = np.cos(heading / 2)

        try:
            resp = self.set_model_srv( state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


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
    traffic = Traffic()

    # Initialise the node
    rospy.init_node('traffic')

    # Set update rate
    r = rospy.Rate(60)
    
    n = 30

    while not rospy.is_shutdown():
        try:
            traffic.set_state(n)
            n += 1
            r.sleep()

        except KeyboardInterrupt:
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()
