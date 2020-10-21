#!/usr/bin/env python
import threading
import sys
import math
import rospy
import numpy as np
import numpy.ma as ma
import matplotlib.pyplot as plt

from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
import sensor_msgs.point_cloud2 as pc2

#from ros_graph_slam.msg import PoseNode, Path2D
# to be replaced by 
from sensor_msgs.msg import LaserScan

class Map(object):

    """ Map class stores occupancy grid as a two dimensional numpy array

    Public instance variables:
        width, height       -- Number of rows and columns in occupancy grid
        resoltuion          -- Width of each grid cell square in meters
        origin_x, origin_y  -- Position of grid cell(0,0) in map coordinate frame
        grid                -- Numpy array
    """

    def __init__(self, dx=0.0, dy=0.0, dyaw=0.0, occupancy=[], prev_grid=[]):
        """ Constructs an empty occupancy grid upon initialization """

        self.origin_x = -15.0
        self.origin_y = -15.0
        self.resolution = 0.3
        self.width = 100 
        self.height = 150 
        self.grid = np.zeros((height, width))

        for n in occupancy:
            
            translate_vect = np.array((-dx, -dy))

            c = np.cos(self.dyaw)
            s = np.sin(self.dyaw)
            R = np.array(((c, -s), (s, c))) 

            


            self.grid = prev_grid[n + translate_vect]

        
        lidar_vect = np.array(((lidar_x), (lidar_y)))

        # Creates rotation matrix given theta
        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        R = np.array(((c, -s), (s, c)))  

        # Vector of point in vehicle frame
        vp = np.array(((point_x), (point_y)))

        rotate = R.dot(vp)                  # rotation to allign with global frame
        transform = rotate + lidar_vect     # translates point to global frame



    def get_occupancy(self):
        return np.nonzero(self.grid)
    
    def to_message(self):
        """ Returns nav_msgs/OccupancyGrid representation of the map """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "base_link"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 0))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).
        """
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            pass    # indicates map too small
        else:
            self.grid[iy, ix] = self.grid[iy, ix] + val
            self.grid[iy, ix] = np.clip(self.grid[iy, ix], 0, 1)

class GridMapping(object):
    
    def __init__(self):

        self.lock = threading.Lock()
        self.scan = None
        self.cg2lidar = 2.34
        self.x = None
        self.y = None
        self.yaw = None
        self.x_prev = None
        self.y_prev = None
        self.yaw_prev = None

        self.origin_x = -15
        self.origin_y = -15
        self.resolution = 0.3
        self.width = 15.0
        self.height = 45.0

        self.gmap = Map()
        
        # Initialise publishers
        self.viz_map_pub = rospy.Publisher('/local_map', OccupancyGrid, latch=True, queue_size=30)

        # Initialise subscribers
        rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb)
        rospy.Subscriber('/laser/scan', LaserScan, self.scan_cb)

    def publish_map(self, gmap):
        """ Publishes map """
        msg = gmap.to_message()
        self.viz_map_pub.publish(msg)
        print('Sent Map')

    def scan_cb(self, data):
        #self.lock.acquire()
        self.scan = data
        #self.lock.release()

    def vehicle_state_cb(self, data):

        self.lock.acquire()

        self.x = data.pose.x
        self.y = data.pose.y
        self.yaw = data.pose.theta

        if self.yaw_prev is None:
            self.x_prev = self.x
            self.y_prev = self.y
            self.yaw_prev = self.yaw

        self.lock.release()

    def transform_map(self):
        dx = self.x - self.x_prev
        dy = self.y - self.y_prev
        dyaw = self.yaw - self.yaw_prev

        self.gmap  = Map(dx, dy, dyaw, self.gmap.get_occupancy, self.gmap.grid)

        self.x_prev = self.x
        self.y_prev = self.y
        self.yaw_prev = self.yaw

    def inverse_range_sensor_model(self):

        # Lidar Properties
        angle_min = self.scan.angle_min
        angle_max = self.scan.angle_max
        range_min = self.scan.range_min
        range_max = self.scan.range_max
        angle_increment = self.scan.angle_increment

        print('Distance forwards = {}'.format(self.scan.ranges[360]))
        
        for i in range(0, len(self.scan.ranges)):
            
            theta = i * angle_increment
            r = self.scan.ranges[i]

            # Determines position of detected point in vehicle frame
            px = r*np.cos(theta)
            py = r*np.sin(theta)

            # Limits all updates to confines of the map
            if px > self.height or px < 0.0 or py > self.width or py < 0.0:
                continue
                
            self.gmap.set_cell(px, py, 0.5)   # Cells Occupied

        self.publish_map(self.gmap)
        
def main():
    """
        The main function.

        It parses the arguments and sets up the GridMapping
    """

    gridmapping = GridMapping()

    rospy.init_node("gridmapping_node")

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            gridmapping.inverse_range_sensor_model()
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")


if __name__ == "__main__":
    main()