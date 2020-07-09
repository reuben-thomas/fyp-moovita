#!/usr/bin/env python

import rospy, os, cubic_spline_planner
import numpy as np
import pandas as pd

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from nav_msgs.msg import Path

frame_id = 'base_link'

#initialization
rospy.init_node('path_planner')
path_planner = rospy.Publisher('/ngeeann_av/path', Path, queue_size=3) 


#Quaternion Conversion
def heading_to_quaternion(heading):
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = np.sin(heading / 2.0)
    quaternion.w = np.cos(heading / 2.0)
    return quaternion

#Converts Cubic Spline to Path and PUblishes it
def create_pub_path(ax, ay, ds, count):
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds)
    target_path = Path()
    target_path.header.frame_id = frame_id
    target_path.header.stamp = rospy.Time.now()
    target_path.header.seq = count

    for n in range(0, len(cx)):
        npose = PoseStamped()
        npose.header.frame_id = frame_id
        npose.header.seq = n
        npose.header.stamp = rospy.Time.now()
        npose.pose.position.x = cx[n]
        npose.pose.position.y = cy[n]
        npose.pose.position.z = 0.0
        npose.pose.orientation = heading_to_quaternion(cyaw[n] + (np.pi / 2.0))
        target_path.poses.append(npose)

    path_planner.publish(target_path)

def walk_up_folder(path, dir_goal='fyp-moovita'):
    dir_path = os.path.dirname(path)
    split_path = str.split(dir_path, '/')     
    counter = 0  
    while (split_path[-1] != dir_goal and counter < 20):
        dir_path = os.path.dirname(dir_path)
        split_path = str.split(dir_path, '/')
        counter += 1
    
    return dir_path

if __name__=="__main__":

    r = rospy.Rate(30) 
    count = 0

    while not rospy.is_shutdown():
        try:
            '''
            # Get targets
            dir_path = os.path.dirname(os.path.abspath(__file__))
            dir_path = walk_up_folder(dir_path)
            df = pd.read_csv(os.path.join(dir_path, 'scripts', 'waypoints.csv'))

            '''
            ax = [100.0, 100.0, 96.0, 90.0, 0.0]
            ay = [18.3, 31.0, 43.0, 47.0, 0.0]
            

            #create path
            create_pub_path(ax, ay, 0.1, count)
            count+=1
            r.sleep()

        except rospy.ServiceException as e:
            rospy.logwarn("path planner node failed:  {0}".format(e))