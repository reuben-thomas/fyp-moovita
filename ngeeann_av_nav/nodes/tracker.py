#!/usr/bin/env python

import rospy, cubic_spline_planner, datetime
import numpy as np
import math

from ngeeann_av_nav.msg import State2D, Path2D
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

class PathTracker:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publishers
        self.tracker_pub = rospy.Publisher('/ngeeann_av/ackermann_cmd', AckermannDrive, queue_size=50)
        self.targets_pub = rospy.Publisher('/ngeeann_av/current_target', Pose2D, queue_size=50)


        # Initialise subscribers
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=50)
        self.path_sub = rospy.Subscriber('/ngeeann_av/path', Path2D, self.path_cb, queue_size=100)
        self.success_sub = rospy.Subscriber('/ngeeann_av/success', String, self.success_cb, queue_size=10)

        # Load parameters
        try:
            self.tracker_params = rospy.get_param("/path_tracker")
            self.frequency = self.tracker_params["update_frequency"]
            self.target_vel = self.tracker_params["target_velocity"]
            self.k = self.tracker_params["control_gain"]
            self.ksoft = self.tracker_params["softening_gain"]
            self.kyaw = self.tracker_params["yawrate_gain"]
            self.max_steer = self.tracker_params["steering_limits"]
            self.cg2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]
        
        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.halfpi = np.pi / 2

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.yaw = None
        self.cx = []
        self.cy = []
        self.cyaw = []
        self.points = 1
        self.targets = None

        # For debugging purposes
        self.fails = 0
        self.target_idx = None
        self.error_front_axle = None
        
    def vehicle_state_cb(self, msg):

        ''' Callback function to receive information on the vehicle's vertical and horizontal coordinates '''

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = math.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        self.yawrate = msg.twist.w

    def path_cb(self, msg):

        ''' Callback function to receive path data from the Local Path Planner '''

        self.cx = []
        self.cy = []
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            ptheta = msg.poses[i].theta
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta)

        self.targets = len(msg.poses)

        print("\nTotal Points: {}".format(len(msg.poses)))
        
    def success_cb(self, msg):

        ''' Unsubscribes from the Local Path Planner when vehicle has completed all waypoints '''

        if msg.data == "Success.":
            self.path_sub.unregister()
            print("\nVehicle has completed all waypoints")

        elif msg.data == "Reached.":
            print("\nVehicle has almost reached the waypoint {}".format(self.points))
            self.points += 1

        else:
            print("\nVehicle has not yet reached the final waypoint.")

    def target_index_calculator(self):

        ''' Calculates the target index and the error of the front axle to the trajectory '''

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * np.sin(self.yaw + self.halfpi)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw + self.halfpi)

        while not self.cx or not self.cy:
            pass

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        if len(dx) == len(dy):
            d = np.hypot(dx, dy) # Find the distance from the front axle to the path
            self.target_idx = np.argmin(d) # Find the shortest distance in the array

            # Project RMS error onto the front axle vector
            front_axle_vec = [-np.cos(self.yaw + np.pi), -np.sin(self.yaw + np.pi)]
            self.error_front_axle = np.dot([dx[self.target_idx], dy[self.target_idx]], front_axle_vec)

            print("\n")
            print("Vehicle speed: {}".format(self.target_vel))
            print("Front axle position (fx, fy): ({}, {})".format(fx, fy))
            print("Target (x, y): ({}, {})".format(self.cx[self.target_idx], self.cy[self.target_idx]))
            
            return self.target_idx, self.error_front_axle

        else:
            self.fails += 1

            print("\n")
            print("Vehicle speed: {}".format(self.target_vel))
            print("Front axle position (fx, fy): ({}, {})".format(fx, fy))

            return self.target_idx, self.error_front_axle

    '''
    def trajectory_yaw_calc(self, target_idx):

        # points ahead / behind
        n = 3

        if ((target_idx - n) == 0):
            return 0.0
        else:
            #3 points created at previous, current, next target by given increment
            x1 = self.cx[target_idx - n]
            y1 = self.cy[target_idx - n]
            x2 = self.cx[target_idx]
            y2 = self.cy[target_idx]
            x3 = self.cx[target_idx + n]
            y3 = self.cy[target_idx + n]

            #define each side of triangle
            a = self.distance_calc(x1, y1, x2, y2)
            b = self.distance_calc(x2, y2, x3, y3)
            c = self.distance_calc(x1, y1, x3, y3)

            #calculate half perimeter
            p = (a + b + c) * 0.5

            #calculate triangle area
            area = math.sqrt(p * (p - a) + (p -b) + (p - c))

            #radius of circle drawn from 3 points
            r_traj = (a * b * c) / (4.0 * area)

            #trajectory yaw rate
            traj_yaw_rate = self.vel / r_traj
            
            if (self.cyaw[target_idx + n] > self.cyaw[target_idx - n]):
                return traj_yaw_rate
            elif (self.cyaw[target_idx + n] < self.cyaw[target_idx - n]):
                return -traj_yaw_rate
            else:
                return 0.0
    '''
    
    def trajectory_yawrate_calc(self, target_idx):

        ''' Calculates the curvator of the path '''

        # lookahead distance in either direction along the path
        target_range = 2

        intervals = 0
        delta_theta = 0.0
        delta_s = 0.0

        start = target_idx - target_range
        end = target_idx + target_range

        for n in range(start, end + 1):

            if (n >= 0) and ((n + 1) < self.targets):

                x1 = self.cx[n]
                y1 = self.cy[n]
                x2 = self.cx[n+1]
                y2 = self.cy[n+1]

                delta_s += self.distance_calc(x1, y1, x2, y2)
                delta_theta = self.cyaw[n + 1] - self.cyaw[n]
                intervals += 1

        # Average values given calculated intervals between points
        delta_theta = delta_theta / intervals
        delta_s = delta_s / intervals

        # Angular velocity calculation
        w = (delta_theta / delta_s) * self.vel

        return w

    def distance_calc(self, x1, y1, x2, y2):

        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)        
        return dist
        
    def publish_current_target(self, x, y):

        current_target = Pose2D()
        current_target.x = x
        current_target.y = y

        self.targets_pub.publish(current_target)

    def stanley_control(self, last_target_idx):

        ''' Calculates the steering angle of the vehicle '''
        
        current_target_idx, error_front_axle = self.target_index_calculator()

        # Ensures no backtracking to already missed targets
        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        self.publish_current_target(self.cx[current_target_idx], self.cy[current_target_idx])

        '''
        # METHOD 1 
        if (((current_target_idx - 3) >= 0) and ((current_target_idx + 3) < self.targets)):
            # yaw_rate_term = self.kyaw * (self.yawrate - self.trajectory_yaw_calc(current_target_idx))
            print("METHOD 1 __________ Measured Yaw Rate = {}, Trajectory Yaw Rate = {}".format(self.yawrate, self.trajectory_yaw_calc(current_target_idx)))
        
        # METHOD 2
        # yaw_rate_term = self.kyaw * (self.yawrate - self.trajectory_yawrate_calc(current_target_idx))
        print("METHOD 2 __________ Measured Yaw Rate = {}, Trajectory Yaw Rate = {}".format(self.yawrate, self.trajectory_yawrate_calc(current_target_idx)))
        '''

        heading_error = self.normalise_angle(self.cyaw[current_target_idx] - self.yaw - self.halfpi)
        crosstrack_error = np.arctan2(self.k * error_front_axle, self.ksoft + self.target_vel)
        sigma_t = heading_error + crosstrack_error

        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer
        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer
        else:
            pass

        print("\n")
        print("Current Target ID: {}".format(current_target_idx))
        print("Heading error = {}".format(heading_error))
        print("Cross-track error = {}".format(crosstrack_error))
        print("Steering error (+-0.95) = {} + {} = {}".format(heading_error, crosstrack_error, sigma_t))
        print("Failure count: {}".format(self.fails))

        return sigma_t, current_target_idx

    def normalise_angle(self, angle):

        while angle > np.pi:
            angle -= 2 * np.pi

        while angle < -np.pi:
            angle += 2 * np.pi

        return angle

    def set_vehicle_command(self, velocity, steering_angle):

        ''' Publishes the calculated steering angle  '''
        
        drive = AckermannDrive()
        drive.speed = velocity
        drive.acceleration = 1.0
        drive.steering_angle = steering_angle
        drive.steering_angle_velocity = 0.0
        self.tracker_pub.publish(drive)

def main():

    ''' Main function to initialise the class and node. '''

    # Time execution
    begin_time = datetime.datetime.now()

    # Initialise the class
    path_tracker = PathTracker()

    # Initialise the node
    rospy.init_node('path_tracker')

    # Set update rate
    r = rospy.Rate(path_tracker.frequency)

    target_idx, _ = path_tracker.target_index_calculator()

    while not rospy.is_shutdown():
        try:
            steering_angle, target_index = path_tracker.stanley_control(target_idx)
            path_tracker.set_vehicle_command(path_tracker.target_vel, steering_angle)
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Execution time: {}".format(datetime.datetime.now() - begin_time))
            print("Shutting down ROS node...")

if __name__ == "__main__":
    main()