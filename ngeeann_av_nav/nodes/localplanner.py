#!/usr/bin/env python

import rospy, os
import numpy as np

from utils.cubic_spline_planner import *
from utils.quintic_polynomial_planner import *
from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from std_msgs.msg import Float32

class CollisionBreak(Exception): 
    pass

class LocalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publishers
        self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)
        self.path_viz_pub = rospy.Publisher('/ngeeann_av/viz_path', Path, queue_size=10)
        #self.collisions_pub = rospy.Publisher('/ngeeann_av/viz_collisions', Path, queue_size=10)
        self.target_vel_pub = rospy.Publisher('/ngeeann_av/target_velocity', Float32, queue_size=10)

        # Initialise subscribers
        self.goals_sub = rospy.Subscriber('/ngeeann_av/goals', Path2D, self.goals_cb, queue_size=10)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)
        self.gridmap_sub = rospy.Subscriber('/map', OccupancyGrid, self.gridmap_cb, queue_size=10)

        # Load parameters
        try:
            self.planner_params = rospy.get_param("/local_path_planner")
            self.frequency = self.planner_params["update_frequency"]
            self.frame_id = self.planner_params["frame_id"]
            self.target_vel_def = self.planner_params["target_velocity"]
            self.car_width = self.planner_params["car_width"]
            self.cg2frontaxle = 1.483

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.halfpi = np.pi / 2
        self.ds = 0.1
        self.origin_x = 0
        self.origin_y = 0

        # Class variables to use whenever within the class when necessary
        self.target_vel = self.target_vel_def
        self.ax = []
        self.ay = []
        self.gmap = OccupancyGrid()
        
        # distance before or after obstacles to deviate and intersect with the original path
        self.react_dist = 75
        
        

    def goals_cb(self, msg):
        ''' Callback function to recieve immediate goals from global planner in global frame'''

        self.ax = []
        self.ay = []
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            self.ax.append(px)
            self.ay.append(py)

        print("\nGoals received: {}".format(len(msg.poses)))

    def vehicle_state_cb(self, msg):
        ''' Callback function to recieve vehicle state information from localization in global frame'''

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta

    def gridmap_cb(self, msg):
        ''' Callback function to recieve map data'''

        self.gmap = msg

    def target_index_calculator(self, cx, cy):  
        ''' Calculates closest point along the path to vehicle front axle'''

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * -np.sin(self.yaw)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw)

        dx = [fx - icx for icx in cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        reference_idx = np.argmin(d) # Find the shortest distance in the array
        return reference_idx

    def determine_path(self, cx, cy, cyaw):
        ''' Map function to validate and determine a path by the following steps:

            1: Identify vehicle's progress along path, search all points ahead for potential collisions
            2: Draft a collision avoidance strategy
        '''

        # Initializing map information
        width = self.gmap.info.width
        height = self.gmap.info.height
        resolution = self.gmap.info.resolution
        origin_x = self.origin_x
        origin_y = self.origin_y
        collisions = []
        collide_id = None

        # Current vehicle progress along path
        lateral_ref_id = self.target_index_calculator(cx, cy)

        #  Validates path of collisions
        for n in range(self.react_dist, len(cyaw) - self.react_dist - 1):

            # Draws side profile of the vehicle along the path ahead
            for i in np.arange(-0.5 * self.car_width, 0.5 * self.car_width, resolution):
                ix = int((cx[n] + i*np.cos(cyaw[n] - 0.5 * np.pi) - origin_x) / resolution)
                iy = int((cy[n] + i*np.sin(cyaw[n] - 0.5 * np.pi) - origin_y) / resolution)
                p = iy * width + ix
                if (self.gmap.data[p] != 0):
                    collisions.append(n)
        
        if len(collisions) != 0:
            
            cx, cy, cyaw = self.collision_avoidance(collisions, cx, cy, cyaw)

        return cx, cy, cyaw, collisions

    def collision_avoidance(self, collisions, cx, cy, cyaw):
        ''' Function that determines the collision avoidance strategy'''

        opening_width = 0
        opening_id = 0
        opening_dist = 0
        collide_view = []

        resolution = self.gmap.info.resolution
        width = self.gmap.info.width
        height = self.gmap.info.height
        origin_x = self.origin_x
        origin_y = self.origin_y

        collide_id = collisions[0]
            
        # Creates collision window, looking 10 meters to the left and right at the point of collision
        for i in np.arange(-10, 10, 0.1):
            ix = int((cx[collide_id] + i*np.cos(cyaw[collide_id] - 0.5 * np.pi) - origin_x) / resolution)
            iy = int((cy[collide_id] + i*np.sin(cyaw[collide_id] - 0.5 * np.pi) - origin_y) / resolution)
            p = iy * width + ix
            collide_view.append(self.gmap.data[p])

        opening_width, opening_id = self.find_opening(collide_view)
        opening_width = opening_width * 0.1
        opening_dist = (opening_id - 100) * 0.1

        if opening_width < self.car_width:
            self.target_vel = 0
        
        else:
            self.target_vel = self.target_vel_def

        print('Obstacle Length: {}'.format(0.1*len(collisions)))
        print('Detected opening of width: {}'.format(opening_width))
        print('Detected distance to opening: {}'.format(opening_dist))

        cx, cy, cyaw = self.collision_reroute(cx, cy, cyaw, collisions, opening_dist)
        return cx, cy, cyaw

    def find_opening(self, arr):
        ''' 
        Recieves an array representing the view perpendicular to the path at the point of predicted collision
        Returns index of midpoint of largest opening, and size
        '''

        count = 0 
        result = 0
        idx = 0

        for i in range(0, len(arr)): 
            if (arr[i] >= 10): #threshold value
                count = 0

            else: 
                count+= 1

                if (count > result):
                    idx = i

                result = max(result, count)

        idx = int(round(idx - result / 2.0))   #midpoint of largest opening
        return result, idx

    def find_closest_opening(self, arr):
        ''' 
        Recieves an array representing the view perpendicular to the path at the point of predicted collision
        Returns index of midpoint of closest viable opening for the vehicle
        '''
        return result, idx

    def collision_reroute(self, cx, cy, cyaw, collisions, opening_dist):

        collide_id = collisions[0]
        collide_id_end = collisions[-1]
        react_dist = self.react_dist

        # Points to leave path
        x_1 = cx[collide_id - react_dist]
        y_1 = cy[collide_id - react_dist]
        yaw_1 = cyaw[collide_id - react_dist]

        # Point of avoidance from collision
        x_2 = cx[collide_id] + opening_dist*np.cos(cyaw[collide_id] - 0.5 * np.pi)
        y_2 = cy[collide_id] + opening_dist*np.sin(cyaw[collide_id] - 0.5 * np.pi)
        yaw_2 = cyaw[collide_id]

        x_3 = cx[collide_id_end] + opening_dist*np.cos(cyaw[collide_id_end] - 0.5 * np.pi)
        y_3 = cy[collide_id_end] + opening_dist*np.sin(cyaw[collide_id_end] - 0.5 * np.pi)
        yaw_3 = cyaw[collide_id_end]

        # Point to intersect path
        x_4 = cx[collide_id_end + react_dist]
        y_4 = cy[collide_id_end + react_dist]
        yaw_4 = cyaw[collide_id_end + react_dist] 

        curve_vel = 0.1

        # Deviation Spline
        _, dev_cx, dev_cy, dev_cyaw, _, _, _ = quintic_polynomials_planner(x_1, y_1, yaw_1, curve_vel, 0.0, x_2, y_2, yaw_2, curve_vel, 0.0, 5.0, 5.0, self.ds)
        # Avoidance Spline
        _, avoid_cx, avoid_cy, avoid_cyaw, _, _, _ = quintic_polynomials_planner(x_2, y_2, yaw_2, curve_vel, 0.0, x_3, y_3, yaw_3, curve_vel, 0.0, 5.0, 5.0, self.ds)
        # Intersection Spline
        _, intersect_cx, intersect_cy, intersect_cyaw, _, _, _ = quintic_polynomials_planner(x_3, y_3, yaw_3, curve_vel, 0.0, x_4, y_4, yaw_4, curve_vel, 0.0, 5.0, 5.0, self.ds)
        # Complete Debug Spline
        # _, debug_cx, debug_cy, debug_cyaw, _, _, _ = quintic_polynomials_planner(x_1, y_1, yaw_1, curve_vel, 0.0, x_4, y_4, yaw_4, curve_vel, 0.0, 5.0, 5.0, self.ds)

        # stiching to form new path
        cx = np.concatenate(( cx[0 : collide_id - react_dist], dev_cx, avoid_cx, intersect_cx, cx[(collide_id_end + react_dist) : ] ))
        cy = np.concatenate(( cy[0 : collide_id - react_dist], dev_cy, avoid_cy, intersect_cy, cy[(collide_id_end + react_dist) : ] ))
        cyaw = np.concatenate(( cyaw[0 : collide_id - react_dist], dev_cyaw, avoid_cyaw, intersect_cyaw, cyaw[(collide_id_end + react_dist) : ] ))

        print('Generated dev path')
        return cx, cy, cyaw

    def create_pub_path(self):
        ''' Uses the cubic_spline_planner library to interpolate a cubic spline path over the given waypoints '''

        # Default direct path drawn across waypoints
        ocx, ocy, ocyaw, _, _ = calc_spline_course(self.ax, self.ay, self.ds)
        # Validated path returned
        cx, cy, cyaw, collisions = self.determine_path(ocx, ocy, ocyaw)

        cells = min(len(cx), len(cy), len(cyaw))
        target_path = Path2D()
        
        viz_path = Path()
        viz_path.header.frame_id = "map"
        viz_path.header.stamp = rospy.Time.now()

        for n in range(0, cells):
            # Appending to Target Path
            npose = Pose2D()
            npose.x = cx[n]
            npose.y = cy[n]
            npose.theta = cyaw[n]
            target_path.poses.append(npose)

            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = self.frame_id
            vpose.header.seq = n
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = cx[n]
            vpose.pose.position.y = cy[n]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = self.heading_to_quaternion(np.pi * 0.5 - cyaw[n])
            viz_path.poses.append(vpose)

        '''
        viz_collisions = Path()
        viz_collisions.header.frame_id = "map"
        viz_collisions.header.stamp = rospy.Time.now()

        for i in collisions:
            vpose = PoseStamped()
            vpose.header.frame_id = self.frame_id
            vpose.header.seq = i
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = ocx[i]
            vpose.pose.position.y = ocy[i]
            vpose.pose.position.z = 0.0
            vpose.pose.orientation = self.heading_to_quaternion(np.pi * 0.5 - ocyaw[i])
            viz_collisions.poses.append(vpose)

            self.collisions_pub.publish(viz_collisions)
        '''

        self.local_planner_pub.publish(target_path)
        self.path_viz_pub.publish(viz_path)

    def heading_to_quaternion(self, heading):

        ''' Converts yaw heading to quaternion coordinates '''

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(heading / 2)
        quaternion.w = np.cos(heading / 2)

        return quaternion

def main():

    ''' Main function to initialise the class and node. '''

    # Initialise the class
    local_planner = LocalPathPlanner()

    # Initialise the node
    rospy.init_node('local_planner')

    # Set update rate
    r = rospy.Rate(local_planner.frequency) 

    # Wait for messages
    rospy.wait_for_message('/ngeeann_av/goals', Path2D)
    rospy.wait_for_message('/map', OccupancyGrid)

    while not rospy.is_shutdown():
        try:
            local_planner.create_pub_path()
            local_planner.target_vel_pub.publish(local_planner.target_vel)

            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()