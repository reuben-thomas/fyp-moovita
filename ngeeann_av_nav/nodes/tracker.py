#!/usr/bin/env python

import rospy, cubic_spline_planner
import numpy as np
import math

from ngeeann_av_nav.msg import State2D
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from ngeeann_av_nav.msg import Path2D

class PathTracker:

    def __init__(self):

        # Initialise publishers
        self.tracker_pub = rospy.Publisher('/ngeeann_av/ackermann_cmd', AckermannDrive, queue_size=50)
        
        # Initialise subscribers
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=50)
        self.path_sub = rospy.Subscriber('/ngeeann_av/path', Path2D, self.path_cb, queue_size=10)

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
        
    def vehicle_state_cb(self, msg):

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = math.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        self.yawrate = msg.twist.w

    def path_cb(self, msg):
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].x
            py = msg.poses[i].y
            ptheta = msg.poses[i].theta
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(ptheta)

        self.targets = len(msg.poses)

        rospy.loginfo("Total Points: {}".format(len(msg.poses)))
        self.path_sub.unregister()
        
    def target_index_calculator(self):

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * np.sin(self.yaw + self.halfpi)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw + self.halfpi)

        while not self.cx or not self.cy:
            pass

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Project RMS error onto the front axle vector
        front_axle_vec = [-np.cos(self.yaw + np.pi), -np.sin(self.yaw + np.pi)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        print("\n")
        print("Vehicle speed: {}".format(self.target_vel))
        print("Front axle position (fx, fy): ({}, {})".format(fx, fy))
        print("Target (x, y): ({}, {})".format(self.cx[target_idx], self.cy[target_idx]))


        # print("e(t): {}".format(error_front_axle))
        
        return target_idx, error_front_axle

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


    def distance_calc(self, x1, y1, x2, y2):

        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        return dist


    def stanley_control(self, last_target_idx):
        
        current_target_idx, error_front_axle = self.target_index_calculator()

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx


        if (((current_target_idx - 3) >= 0) and ((current_target_idx + 3) < self.targets)):
            yaw_rate_term = self.kyaw * (self.yawrate - self.trajectory_yaw_calc(current_target_idx))
            print("Measured Yaw Rate = {}, Trajectory Yaw Rate = {}".format(self.yawrate, yaw_rate_term))

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
        print("Heading error = {}".format(heading_error))
        print("Cross-track error = {}".format(crosstrack_error))
        print("Steering error (+-0.95) = {} + {} = {}".format(heading_error, crosstrack_error, sigma_t))

        return sigma_t, current_target_idx

    def normalise_angle(self, angle):

        while angle > np.pi:
            angle -= 2 * np.pi

        while angle < -np.pi:
            angle += 2 * np.pi

        return angle

    def set_vehicle_command(self, velocity, steering_angle):
        
        drive = AckermannDrive()
        drive.speed = velocity
        drive.acceleration = 1.0
        drive.steering_angle = steering_angle
        drive.steering_angle_velocity = 0.0
        self.tracker_pub.publish(drive)

def main():

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
            rospy.loginfo("Shutting down ROS node...")

if __name__ == "__main__":
    main()