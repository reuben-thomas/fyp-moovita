#!/usr/bin/env python

#LOCALIZATION NODE
#Retrieves vehicle state from gazebo, converts to 2D position and planae

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose2D
import numpy as np


#initialization
rospy.init_node('localisation')
rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state') 
get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)
localisation = rospy.Publisher('/ngeeann_av/state2D',Pose2D, queue_size=1) 


def show_vehicle_status():
    print('\n\nPosition:')
    print('x: ' + str(state.pose.position.x))
    print('y: ' + str(state.pose.position.y))
    print('z: ' + str(state.pose.position.z))
    yaw = get_yaw_rad()
    print('Heading: ' + str(yaw))


def get_heading():
    heading = 2.0 * np.arctan2(state.pose.orientation.z, state.pose.orientation.w)
    return heading


def update_state():
    state2d = Pose2D()
    state2d.x = state.pose.position.x
    state2d.y = state.pose.position.y
    state2d.theta = get_heading()
    localisation.publish(state2d)


r = rospy.Rate(30) #Set update rate, default to 30

if __name__=="__main__":

    while not rospy.is_shutdown():
        try:
            state = get_model_srv('ngeeann_av', '')
            update_state()
            r.sleep()
        except rospy.ServiceException as e:
            rospy.loginfo("Navigation node failed:  {0}".format(e))