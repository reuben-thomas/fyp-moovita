#!/usr/bin/env python
import threading
import sys
import math
import rospy
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
import sensor_msgs.point_cloud2 as pc2

class CruiseControl(object):

    self.lock = threading.Lock()
    self.scan = None
    self.cg2lidar = 2.34
    self.x = None
    self.y = None
    self.yaw = None

    self.gmap = Map()  

    # Initialise subscribers
    rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb)
    rospy.Subscriber('/laser/scan', LaserScan, self.scan_cb)
    rospy.Subscriber('/ngeeann_av/path', Path2D, self.path_cb, queue_size=10)

    def vehicle_state_cb(self, data):
        # Fill gridmap
        self.lock.acquire()
        self.x = data.pose.x
        self.y = data.pose.y
        self.vel = np.sqrt(data.twist.x**2 + data.twist.y**2)
        self.yaw = data.pose.theta
        self.lock.release()

    def scan_cb(self, data):
        self.lock.acquire()
        self.scan = data
        self.lock.release()

    def path_cb(self, msg):
        self.lock.acquire()
        self.cx = []
        self.cy = []
        self.cyaw = []

        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            ptheta = msg.poses[i].theta
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta) 
        self.lock.release()

    def inverse_range_sensor_model(self):

        # Lidar Properties
        angle_min = self.scan.angle_min
        angle_max = self.scan.angle_max
        range_min = self.scan.range_min
        range_max = self.scan.range_max
        angle_increment = self.scan.angle_increment

        print('Distance forwards = {}'.format(self.scan.ranges[400]))



            
    def frame_transform(self, point_x, point_y):
        ''' 
            Recieves position of a point in the vehicle frame, and the position and orientation of the
            vehicle in the global frame. Returns position of the point in global frame

            Arguments:
                point_x, point_y   - Coordinates (x,y) of point in the vehicle frame
                self.x, self.y     - Coordinates (x,y) of vehicle centre of gravity in the world frame
                self.yaw           - Yaw angle of the vehicle respect to global frame
                self.cg2lidar      - Distance between the centre of gravity of the vehicle and lidar module
        '''
        # Lidar position in world frame
        lidar_x = self.x + self.cg2lidar * -np.sin(self.yaw)
        lidar_y = self.y + self.cg2lidar * np.cos(self.yaw)
        lidar_vect = np.array(((lidar_x), (lidar_y)))

        # Creates rotation matrix given theta
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        R = np.array(((c, -s), (s, c)))  

        # Vector of point in vehicle frame
        vp = np.array(((point_x), (point_y)))

        rotate = R.dot(vp)                  # rotation to allign with global frame
        transform = rotate + lidar_vect     # translates point to global frame

        return transform 

