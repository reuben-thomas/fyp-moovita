
#! /usr/bin/env python
import rospy
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Quaternion
from ngeeann_av_nav.msg import State2D
import time
import math
from tf.transformations import quaternion_from_euler

    # Initialise publishers
    self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)

    # Initialise subscribers
    self.goals_sub = rospy.Subscriber('/ngeeann_av/', Path2D, self.goals_cb, queue_size=10)



def callback(data):
    odom = Odometry()
    odom.pose.pose.position.x = data.pose.x
    odom.pose.pose.position.y = data.pose.y
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "/map"
    
    q = quaternion_from_euler(0, 0, data.pose.theta)
    odom.pose.pose.orientation = Quaternion(*q)
    global pub
    pub.publish(odom)

rospy.init_node('listener', anonymous=True)
pub = rospy.Publisher('/odom', Odometry, queue_size=10)
rospy.Subscriber("/pose2D", Pose2D, callback)
rospy.Subscriber("/ngeeann_av/state2D", State2D, callback)

rospy.spin()