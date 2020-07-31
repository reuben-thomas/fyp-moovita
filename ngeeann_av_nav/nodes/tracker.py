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
        
        # Initialise publishers
        self.tracker_pub = rospy.Publisher('/ngeeann_av/ackermann_cmd', AckermannDrive, queue_size=10)
        self.targets_pub = rospy.Publisher('/ngeeann_av/current_target', Pose2D, queue_size=10)

        # Initialise subscribers
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb, queue_size=10)
        self.path_sub = rospy.Subscriber('/ngeeann_av/path', Path2D, self.path_cb, queue_size=10)
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
        self.totalerror = []
        self.cyaw = []
        self.points = 1
        self.targets = None

        # For debugging purposes
        self.fails = 0
        self.target_idx = None
        self.error_front_axle = None
        self.heading_error = None
        self.yawrate_error = None


    # Callback function to recieve information on the vehicle's vertical and horizontal coordinates
    def vehicle_state_cb(self, msg):

        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = math.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        self.yawrate = msg.twist.w
        self.target_index_calculator()


    # Callback function to recieve path data from the local path planner
    def path_cb(self, msg):

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
        '''
        
        # Debug Circle Path
        ax = [103.67,103.18603057117863,101.73864097687299,99.34134510374726,96.01652585745269,91.79522617946827,86.7168592075202,80.82884028573427,
                74.18614426034304,66.8507921943588,58.891272293618925,50.38190045086097,41.402126378255645,32.03579180685079,22.37034767891844,12.496037642069313,2.5050554686298017]
        ay = [0.0,10.00559818120778,19.917776788058948,29.643988479592114,39.09342223782818,48.17785124577748,56.81245663745863,64.91661942879698,
                72.4146732353573,79.2366107489881,85.31873737719536,90.60426594239112,95.0438468884701,98.59602904431858,101.22764664223064,102.91412897774526,103.6397298196936]

        self.cx, self.cy, self.cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
        '''


    # Unsubscribes from local path planner once vehicle has completed all waypoints
    def success_cb(self, msg):

        if msg.data == "Success.":
            self.path_sub.unregister()
            print("\nVehicle has completed all waypoints")

        elif msg.data == "Reached.":
            print("\nVehicle has almost reached the waypoint {}".format(self.points))
            self.points += 1

        else:
            print("\nVehicle has not yet reached the final waypoint.")

    
    # Calculates the target index and each corresponding error
    def target_index_calculator(self):

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * np.sin(self.yaw + self.halfpi)
        fy = self.y + self.cg2frontaxle * np.cos(self.yaw + self.halfpi)

        while not self.cx or not self.cy:
            pass

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        if len(dx) != len(dy):
            self.fails += 1
            pass

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        self.target_idx = np.argmin(d) # Find the shortest distance in the array

        # Cross track error, project RMS error onto the front axle vector
        front_axle_vec = [-np.cos(self.yaw + np.pi), -np.sin(self.yaw + np.pi)]
        self.error_front_axle = np.dot([dx[self.target_idx], dy[self.target_idx]], front_axle_vec)
    
        # Heading error
        self.heading_error = self.normalise_angle(self.cyaw[self.target_idx] - self.yaw - self.halfpi)

        # Yaw rate discrepancy
        #self.yawrate_error = self.trajectory_yawrate_calc() - self.yawrate

        print("\n")
        print("Vehicle speed: {}".format(self.target_vel))
        print("Front axle position (fx, fy): ({}, {})".format(fx, fy))
        print("Target (x, y): ({}, {})".format(self.cx[self.target_idx], self.cy[self.target_idx]))

    
    # Calculates the desired yawrate of the vehicle
    def trajectory_yawrate_calc(self):
        target_range = 2    #number of points to look ahead and behind
        intervals = 0
        delta_theta = 0.0
        delta_s = 0.0
        w = 0.0

        start = self.target_idx - target_range
        end = self.target_idx + target_range

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
        if (intervals > 0):
            delta_theta = delta_theta / intervals
            delta_s = delta_s / intervals

            # Angular velocity calculation
            w = (delta_theta / delta_s) * self.vel

        return w


    # Calculates distance between two points in 2D
    def distance_calc(self, x1, y1, x2, y2):

        dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)        
        return dist


    # Publishes current vehicle target
    def publish_current_target(self, x, y):

        current_target = Pose2D()
        current_target.x = x
        current_target.y = y

        self.targets_pub.publish(current_target)


    # Stanley controller determines the appropriate steering angle
    def stanley_control(self, last_target_idx):

        if (self.target_idx == None):
            self.target_index_calculator()

        # Ensures no backtracking to already missed targets
        if last_target_idx >= self.target_idx:
            self.target_idx = last_target_idx

        print("cx:{}\ntargets:{}".format(len(self.cx), self.target_idx))
        self.publish_current_target(self.cx[self.target_idx], self.cy[self.target_idx])
        
        crosstrack_term = np.arctan2(self.k * self.error_front_axle, self.ksoft + self.target_vel)
        heading_term = self.heading_error
        # yawrate_term = self.kyaw * self.yawrate_error

        sigma_t = crosstrack_term + heading_term

        # Constrains steering angle to the vehicle limits
        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer
        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer
        else:
            pass

        return sigma_t, self.target_idx


    # Normalises angle to -pi to pi
    def normalise_angle(self, angle):

        while angle > np.pi:
            angle -= 2 * np.pi

        while angle < -np.pi:
            angle += 2 * np.pi

        return angle


    # Publishes to vehicle state
    def set_vehicle_command(self, velocity, steering_angle):

        ''' Publishes the calculated steering angle  '''
        
        drive = AckermannDrive()
        drive.speed = velocity
        drive.acceleration = 1.0
        drive.steering_angle = steering_angle
        drive.steering_angle_velocity = 0.0
        self.tracker_pub.publish(drive)

def main():
    # Time execution
    begin_time = datetime.datetime.now()

    # Initialise the class
    path_tracker = PathTracker()

    # Initialise the node
    rospy.init_node('path_tracker')

    # Set update rate
    r = rospy.Rate(path_tracker.frequency)

    while not rospy.is_shutdown():
        try:
            steering_angle, path_tracker.target_idx = path_tracker.stanley_control(path_tracker.target_idx)
            path_tracker.set_vehicle_command(path_tracker.target_vel, steering_angle)

            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Execution time: {}".format(datetime.datetime.now() - begin_time))
            print("Shutting down ROS node...")

if __name__ == "__main__":
    main()
