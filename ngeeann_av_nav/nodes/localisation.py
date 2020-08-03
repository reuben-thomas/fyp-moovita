#!/usr/bin/env python

import rospy
import numpy as np
import tf

from gazebo_msgs.srv import GetModelState  
from nav_msgs.msg import Odometry
from ngeeann_av_nav.msg import State2D


class Localisation:

    def __init__(self):

        # Wait and initialise service
        rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state') 
        self.get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)

        # Initialise publishers
        self.localisation_pub = rospy.Publisher('/ngeeann_av/state2D', State2D, queue_size=10)

        # Publishes artificial map frame
        self.map_broadcaster = tf.TransformBroadcaster()

        # Load parameters
        try:
            self.localisation_params = rospy.get_param("/localisation")
            self.frequency = self.localisation_params["update_frequency"]
            self.model = self.localisation_params["model_name"]

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state = None


    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.position.x
        state2d.pose.y = self.state.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.orientation.z, self.state.pose.orientation.w)

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
        print("Position (x,y): ({},{})".format(round(state2d.pose.x, 5), round(state2d.pose.y, 5)))
        print("Heading: {}".format(round(state2d.pose.theta, 5)))
        print("Velocity (x,y): ({},{})".format(round(state2d.twist.x, 5), round(state2d.twist.y, 5)))


    # Publishes map frame transform
    def update_odom(self):

        # Publish transform
        x = self.state.pose.position.x
        y = self.state.pose.position.y
        z = self.state.pose.position.z
        odom_quat = [self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w]
        self.map_broadcaster.sendTransform((x, y, z), odom_quat, rospy.Time.now(), "base_link", "map")


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
            localisation.update_odom()
            r.sleep()

        except KeyboardInterrupt:
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()