#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('navigation')

rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state') # Wait for service to start
get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)

r = rospy.Rate(2)

while not rospy.is_shutdown():
    try:
        state = get_model_srv('ngeeann_av', 'ground')
        print 'Status.success', state.success
        print('\n\nPOSITION:')
        print('x: ' + str(state.pose.position.x))
        print('y: ' + str(state.pose.position.y))
        print('z: ' + str(state.pose.position.z))
        print('ORIENTATION:')
        print('x: ' + str(state.pose.orientation.x))
        print('y: ' + str(state.pose.orientation.y))
        print('z: ' + str(state.pose.orientation.z))
        r.sleep()
    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))
