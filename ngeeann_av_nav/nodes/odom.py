#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('/odom', Odometry)

rospy.wait_for_service('/gazebo/get_model_state') # Wait for service to start
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# Empty odometry message
odom = Odometry()

# Empty header which holds the time stamps and sequence
header = Header()
header.frame_id = '/odom'

model = GetModelStateRequest()
model.model_name = 'ngeeann_av'

r = rospy.Rate(2)

while not ropsy.is_shutdown():
    result = get_model_srv(model)
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)

    r.sleep()
