#!/usr/bin/env python

# LOCALISATION NODE
# Retrieves vehicle state from gazebo, converts to 2D position and planae

import rospy
import numpy as np

from gazebo_msgs.srv import GetModelState  
from ngeeann_av_nav.msg import State2D

class Localisation:

    def __init__(self):

        # Wait and initialise service
        rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state') 
        self.get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)

        # Initialise publishers
        self.localisation_pub = rospy.Publisher('/ngeeann_av/state2D', State2D, queue_size=50)

        # Load parameters
        self.localisation_params = rospy.get_param("/localisation")
        self.frequency = self.localisation_params["update_frequency"]
        self.model = self.localisation_params["model_name"]

        # Class constants
        self.state = None

    def update_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.position.x
        state2d.pose.y = self.state.pose.position.y
        state2d.pose.theta = -2.0 * np.arctan2(self.state.pose.orientation.z, self.state.pose.orientation.w)

        # Aligning heading to y-axis
        if state2d.pose.theta > 2.0 * np.pi:
            state2d.pose.theta -= 2.0 * np.pi
        elif state2d.pose.theta < 0.0:
            state2d.pose.theta += 2.0 * np.pi

        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.linear.x
        state2d.twist.y = self.state.twist.linear.y
        state2d.twist.w = -self.state.twist.angular.z


        self.localisation_pub.publish(state2d)

        # Print state
        rospy.loginfo("Position (x,y): ({},{})".format(round(state2d.pose.x, 5), round(state2d.pose.y, 5)))
        rospy.loginfo("Heading: {}".format(round(state2d.pose.theta, 5)))

def main():

    # Initialise the class
    localisation = Localisation()

    # Initialise the node
    rospy.init_node('localisation')

    # Set update rate
    r = rospy.Rate(localisation.frequency)
    
    while not rospy.is_shutdown():
        try:
            localisation.state = localisation.get_model_srv(localisation.model, '')
            localisation.update_state()
            r.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down ROS node...")

if __name__=="__main__":
    main()