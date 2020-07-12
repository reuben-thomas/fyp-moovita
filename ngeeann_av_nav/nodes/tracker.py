#!/usr/bin/env python

import rospy, cubic_spline_planner
import numpy as np

from geometry_msgs.msg import Pose2D
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path

class PathTracker:

    def __init__(self):

        # Initialise publishers
        self.tracker_pub = rospy.Publisher('/ngeeann_av/ackermann_cmd', AckermannDrive, queue_size=50)
        
        # Initialise subscribers
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', Pose2D, self.vehicle_state_cb, queue_size=50)
        self.path_sub = rospy.Subscriber('/ngeeann_av/path', Path, self.path_cb, queue_size=10)

        # Load parameters
        self.tracker_params = rospy.get_param("/path_tracker")
        self.frequency = self.tracker_params["update_frequency"]
        self.target_vel = self.tracker_params["target_velocity"]
        self.k = self.tracker_params["control_gain"]
        self.ksoft = self.tracker_params["softening_gain"]
        self.max_steer = self.tracker_params["steering_limits"]
        self.cg2frontaxle = self.tracker_params["centreofgravity_to_frontaxle"]

        # Class constants
        self.halfpi = np.pi / 2.0

        # Class variables to use whenever within the class when necessary
        self.x = None
        self.y = None
        self.yaw = None
        self.cx = []
        self.cy = []
        self.cyaw = []
        
    def vehicle_state_cb(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.yaw = msg.theta

    def path_cb(self, msg):
        
        for i in range(0, len(msg.poses)):
            px = msg.poses[i].pose.position.x
            py = msg.poses[i].pose.position.y
            orientation = 2.0 * np.arctan2(msg.poses[i].pose.orientation.z, msg.poses[i].pose.orientation.w)
            self.cx.append(px)
            self.cy.append(py)
            self.cyaw.append(orientation)

        self.path_sub.unregister()
        
    def target_index_calculator(self):

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * np.cos(self.yaw)
        fy = self.y + self.cg2frontaxle * np.sin(self.yaw)

        while not self.cx or not self.cy:
            pass

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Project RMS error onto the front axle vector
        front_axle_vec = [-np.cos(self.yaw + self.halfpi), -np.sin(self.yaw + self.halfpi)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        print("\n")
        print("Vehicle speed: {}".format(self.target_vel))
        print("Front axle position (fx, fy): ({}, {})".format(fx, fy))
        print("Target (x, y): ({}, {})".format(self.cx[target_idx], self.cy[target_idx]))
        print("e(t): {}".format(error_front_axle))
        
        return target_idx, error_front_axle

    def stanley_control(self, last_target_idx):
        
        current_target_idx, error_front_axle = self.target_index_calculator()

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        heading_error = self.normalise_angle(self.cyaw[current_target_idx] - self.yaw)
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
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

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