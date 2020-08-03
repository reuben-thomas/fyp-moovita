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

    def __init__(self, origin_x=-250, origin_y=-250, resolution=.1, width=5000, height=5000):
        """ Constructs an empty occupancy grid upon initialization """

        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height 
        self.grid = np.zeros((height, width))

    
    def to_message(self):
        """ Returns nav_msgs/OccupancyGrid representation of the map """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        """ log odds conversion required """

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
            print("Map to small.")
        self.grid[iy, ix] = min(1.0, self.grid[iy, ix] + val)




class GridMapping(object):
    
    def __init__(self):

        self.lock = threading.Lock()
        self.scan = None

        rospy.init_node("gridmapping_node")
        
        # Initialise publishers
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=10)
        self._map_data_pub = rospy.Publisher('map_metadata', MapMetaData, latch=True, queue_size=10)

        # Initialise subscribers
        rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb)
        #rospy.Subscriber('/ngeeann_av/path', Path2D, self.path_cb)
        rospy.Subscriber('/laser/scan', LaserScan, self.scan_cb)

    def publish_map(self, gmap):
        """ Publishes map """
        grid_msg = gmap.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)

    def scan_cb(self, data):
        self.lock.acquire()
        self.scan = data
        self.lock.release()

    def vehicle_state_cb(self, data):
        # Fill gridmap
        self.lock.acquire()
        gmap = Map()

        angle_min = self.scan.angle_min
        angle_max = self.scan.angle_max
        range_min = self.scan.range_min
        range_max = self.scan.range_max
        angle_increment = self.scan.angle_increment

        x = data.pose.x
        y = data.pose.y
        yaw = data.pose.theta


        print("Minimum Angle: {}\nMaximum Angle{}".format(angle_min, angle_max))

        self.lock.release()



def main():
    """
        The main function.

        It parses the arguments and sets up the GridMapping
    """
    GridMapping()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    sys.exit(0)

if __name__ == "__main__":
    main()
