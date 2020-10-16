#!/usr/bin/env python

import rospy, os
import numpy as np

from utils.cubic_spline_planner import *
from utils.quintic_polynomial_planner import *
from geometry_msgs.msg import PoseStamped, Quaternion, Pose2D, Point
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import Path, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class CollisionBreak(Exception): 
    pass

class LocalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publishers
        self.local_planner_pub = rospy.Publisher('/ngeeann_av/path', Path2D, queue_size=10)
        self.path_viz_pub = rospy.Publisher('/ngeeann_av/viz_path', Path, queue_size=10)
        self.node_pub = rospy.Publisher('ngeeann_av/lp_nodes', Marker, queue_size = 10)
        self.collisions_pub = rospy.Publisher('/ngeeann_av/viz_collisions', Path, queue_size=10)
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
        ''' Map function to validate and determine a path by the following steps. Method inspired by:
            https://ri.cmu.edu/pub_files/2013/6/IV2013-Tianyu.pdf [Chapter III, Part A. Road Blockage Detection & Seeding Path Generation]

            1: Draw vehicle swath along path to detect collisions
            2: Create node grid along the path from a point prior to collision
            3: Create and validate path (Greedy Algorithm)
        '''
        # Initializing map parameters
        width = self.gmap.info.width
        height = self.gmap.info.height
        resolution = self.gmap.info.resolution
        origin_x = self.origin_x
        origin_y = self.origin_y
        collisions = []
        collide_id = None
        current_target = self.target_index_calculator(cx, cy)

        #  Step 1: Drawing vehicle profile along path to detect collisions
        for n in range(self.react_dist, len(cyaw) - self.react_dist - 1):
            for i in np.arange(-0.5 * self.car_width, 0.5 * self.car_width, resolution):
                ix = int((cx[n] + i*np.cos(cyaw[n] - 0.5 * np.pi) - origin_x) / resolution)
                iy = int((cy[n] + i*np.sin(cyaw[n] - 0.5 * np.pi) - origin_y) / resolution)
                p = iy * width + ix
                if (self.gmap.data[p] != 0):
                    collisions.append(n)
    
        if len(collisions) != 0 and (current_target - self.react_dist) <= collisions[-5]:
            cx, cy, cyaw, collisions = self.collision_avoidance(collisions, cx, cy, cyaw)

        return cx, cy, cyaw, collisions

    def collision_avoidance(self, collisions, ocx, ocy, ocyaw):

        # Initializing map parameters
        width = self.gmap.info.width
        height = self.gmap.info.height
        resolution = self.gmap.info.resolution
        origin_x = self.origin_x
        origin_y = self.origin_y

        delta_L = 2.0       # Interval distance of sampling origin path [m]
        delta_o = 0.5       # Lateral offset between nodes of each layer [m]
        max_o = 10          # Maximum lateral offset in either direction [m]
        occ_thresh = 20     # Occupancy threshold

        dist_weight = 1.0
        offset_weight = 1.0
        occ_weight = 100.0

        dev_id = collisions[0] - self.react_dist
        intersect_id = collisions[-1] + int(delta_L / self.ds) + self.react_dist
        
        # Coordinates of points of deviation before collision period
        ax = [(ocx[dev_id])]
        ay = [(ocy[dev_id])]

        # Clear node visualization
        self.display_node(0,0,0,0,1)

        # Sample points along path delta L
        for n in range(collisions[0], collisions[-1] + int(delta_L / self.ds)):
            if n%int(delta_L / self.ds) == 0:

                # Arrays describing the x,y coordinates of the nodes on layer for this
                nx = []
                ny = []
                n_occ = np.array([])
                n_offset = np.array([])

                # Construct layer of nodes
                for i in np.arange(-max_o, max_o, delta_o):
                    x = (ocx[n] + i*np.cos(ocyaw[n] - 0.5 * np.pi) - origin_x)
                    y = (ocy[n] + i*np.sin(ocyaw[n] - 0.5 * np.pi) - origin_y)
                    ix = int(x / resolution)
                    iy = int(y / resolution)
                    p = iy * width + ix
                    self.display_node(x, y, p, 0, 0)

                    nx.append(x)
                    ny.append(y)
                    n_occ = np.append(n_occ, self.gmap.data[p])
                    n_offset = np.append(n_offset, abs(i))

                dx = [ax[-1] - icx for icx in nx]
                dy = [ay[-1] - icy for icy in ny] 

                dist_cost = dist_weight * np.hypot(dx, dy) 
                occ_cost = occ_weight * n_occ
                offset_cost = offset_weight * n_offset
                total_cost = dist_cost + occ_cost + offset_cost
                
                # Determines the next node by the greedy method
                order = np.argsort(total_cost)
                for q in order:
                    if self.edge_verified_free(ax[-1], ay[-1], nx[q], ny[q]):
                        ax.append(nx[q])
                        ay.append(ny[q])
                        break

        # If the final link to point of intersection not viable, collision window extended and node grid reconstructed
        if self.edge_verified_free(ax[-1], ay[-1], ocx[intersect_id], ocy[intersect_id]):
            ax.append(ocx[intersect_id])
            ay.append(ocy[intersect_id])
            ocx, ocy, ocyaw = self.collision_reroute(ocx, ocy, ocyaw, ax, ay, dev_id, intersect_id)
        else:
            pass
            collisions.append(collisions[-1] + int(delta_L / self.ds))
            self.collision_avoidance(collisions, ocx, ocy, ocyaw)
        
        return ocx, ocy, ocyaw, collisions

    def collision_reroute(self, cx, cy, cyaw, ax, ay, dev_id, intersect_id):
        curve_vel = 0.1
        # Avoidance Spline
        if (len(ax) >= 4):
                avoid_cx, avoid_cy, avoid_cyaw, _, _ = calc_spline_course(ax[1:-1], ay[1:-1], self.ds)
        else:
            # Draws single point when cubic spline unavailable due to insufficient points
            avoid_cx = [ax[1]]
            avoid_cy = [ay[1]]
            avoid_cyaw = [cyaw[dev_id + self.react_dist]]

        # Deviation Quintic Spline
        _, dev_cx, dev_cy, dev_cyaw, _, _, _ = quintic_polynomials_planner(cx[dev_id], cy[dev_id], cyaw[dev_id], curve_vel, 0.0, avoid_cx[0], avoid_cy[0], avoid_cyaw[0], curve_vel, 0.0, 5.0, 5.0, self.ds)
        # Intersection Quintic Spline
        _, intersect_cx, intersect_cy, intersect_cyaw, _, _, _ = quintic_polynomials_planner(avoid_cx[-1], avoid_cy[-1], avoid_cyaw[-1], curve_vel, 0.0, cx[intersect_id], cy[intersect_id], cyaw[intersect_id], curve_vel, 0.0, 5.0, 5.0, self.ds)

        # stiching to form new path
        cx = np.concatenate(( cx[0 : dev_id], dev_cx, avoid_cx, intersect_cx, cx[intersect_id : ] ))
        cy = np.concatenate(( cy[0 : dev_id], dev_cy, avoid_cy, intersect_cy, cy[intersect_id : ] ))
        cyaw = np.concatenate(( cyaw[0 : dev_id], dev_cyaw, avoid_cyaw, intersect_cyaw, cyaw[intersect_id : ] ))
        return cx, cy, cyaw
                
    def straight_path_planner(self, x1, y1, x2, y2):
        yaw = np.arctan2((y2-y1), (x2-x1))
        dist = np.hypot((x2 - x1),(y2 - y1))

        cx = []
        cy = []
        cyaw = []

        for n in np.arange(0, dist, self.ds):
            cx.append(x1 + n * np.cos(yaw))
            cy.append(y1 + n * np.sin(yaw))
            cyaw.append(yaw)

        return cx, cy, cyaw

    def edge_verified_free(self, x1, y1, x2, y2):
        ''' Verifies if an edge betweeen two nodes is occupied or free of collisions '''

        width = self.gmap.info.width
        height = self.gmap.info.height
        resolution = self.gmap.info.resolution
        origin_x = self.origin_x
        origin_y = self.origin_y
        collisions = []
        occ_thresh = 45

        yaw = np.arctan2((y2-y1), (x2-x1))
        dist = np.hypot((x2-x1),(y2-y1)) + 0.5 * self.car_width
        #print('distance is {}'.format(dist))

        for n in np.arange(0, dist, self.ds):
            
            x = x1 + n * np.cos(yaw)
            y = y1 + n * np.sin(yaw)

            for i in np.arange(-0.5 * self.car_width, 0.5 * self.car_width, 0.5*resolution):
                ix = int((x + i*np.cos(yaw - 0.5 * np.pi) - origin_x) / resolution)
                iy = int((y + i*np.sin(yaw - 0.5 * np.pi) - origin_y) / resolution)
                p = iy * width + ix
                if (self.gmap.data[p] >= occ_thresh):
                    collisions.append(1)
                    break

        if len(collisions) > 0:
            # print('point ({}, {}) to point ({}, {}), FAILED'.format(x1, y1, x2, y2))
            return False
        else:
            # print('point ({}, {}) to point ({}, {}), CLEAR'.format(x1, y1, x2, y2))
            self.display_node(x1, y1, p + 1, 1, 0)
            self.display_node(x2, y2, p, 1, 0)
            return True

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
    
    def display_node(self, x, y, idx, color, clear):

        points = Marker()		
        points.header.frame_id = "map"	# publish path in map frame		
        points.type = points.POINTS
        if (clear != 0):
            points.action = points.DELETEALL
        else:
            points.action = points.ADD
            points.lifetime = rospy.Duration(0)
            points.id = idx
            points.scale.x = 0.1
            points.scale.y = 0.1	
            points.color.a = 0.5 + (0.5*color)
            points.color.r = 0.0 + color
            points.color.g = 1.0 - color
            points.color.b = 1.0 - color
            points.pose.orientation.w = 1.0

            point = Point()
            point.x = x
            point.y = y
            points.points.append(point);
        self.node_pub.publish(points)


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