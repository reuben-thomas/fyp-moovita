#!/usr/bin/env python

import rospy, cubic_spline_planner
from geometry_msgs.msg import Pose2D
from ackermann_msgs.msg import AckermannDrive
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Path

target_vel = 5.0 # Target Velocity 
k = 1.0 # Control Gain
ksoft = 1.0 # Softening gain to ensure a non-zero denominator
max_steer = 0.95  # Max steering angles in radians
cog2frontaxle = 1.483 # Distance from the vehicle's centre of gravity to its front axle
halfpi = np.pi / 2


def main():
    path_tracker = rospy.Subscriber('/ngeeann_av/path', Path, queue_size=3)
    localisation = rospy.Subscriber('/ngeeann_av/state2D', Pose2D, vehicle_state)

    rospy.init_node('path_tracker')
    r = rospy.Rate(30) # Set update rate, default to 30

    while not rospy.is_shutdown():
        try:
            test() 
            r.sleep()

        except rospy.ServiceException as e:
            rospy.loginfo("Path tracking node failed:  {0}".format(e))

def vehicle_state(coordinates):
    x = coordinates.x
    y = coordinates.y
    theta = coordinates.theta
    
    return x, y, theta

def test():
    print(xvehicle_state())

if __name__=="__main__":
    main()
