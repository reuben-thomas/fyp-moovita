#!/usr/bin/env python

import rospy, cubic_spline_planner, datetime
import numpy as np
import threading

from gazebo_msgs.srv import GetModelState
from ngeeann_av_nav.msg import State2D, Path2D
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String

class PathTracker:

    def __init__(self):
        
        # Initialise publishers
        self.tracker_pub = rospy.Publisher('/ngeeann_av/ackermann_cmd', AckermannDrive, queue_size=10)

        # Initialise subscribers
        rospy.wait_for_message('/ngeeann_av/state2D', State2D)
        rospy.wait_for_message('/ngeeann_av/path', Path2D)
        self.localisation_sub = rospy.Subscriber('/ngeeann_av/state2D', State2D, self.vehicle_state_cb)
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
        self.x = 100.0
        self.y = 0.0
        self.yaw = 0.0
        self.points = 1
        self.lock = threading.Lock()
        self.cx = []
        self.cy = []
        self.cyaw = []

        self.target_idx = None
        self.heading_error = 0.0
        self.crosstrack_error = 0.0
        self.yawrate_error = 0.0
   
        '''
        # Debug Circle Paths
        ax = [103.67,103.18603057117863,101.73864097687299,99.34134510374726,96.01652585745269,91.79522617946827,86.7168592075202,80.82884028573427,74.18614426034304,66.8507921943588]
        ay = [0.0,10.00559818120778,19.917776788058948,29.643988479592114,39.09342223782818,48.17785124577748,56.81245663745863,64.91661942879698,72.4146732353573,79.2366107489881]
        self.cx, self.cy, self.cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
        '''

    def vehicle_state_cb(self, msg):
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.yaw = msg.pose.theta
        self.vel = np.sqrt((msg.twist.x**2.0) + (msg.twist.y**2.0))
        self.yawrate = msg.twist.w

        if(self.cyaw != []):
            self.target_index_calculator()

    def path_cb(self, msg):
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

    # Calculates the target index and each corresponding error
    def target_index_calculator(self):  

        # Calculate position of the front axle
        fx = self.x + self.cg2frontaxle * np.cos(self.yaw + self.halfpi)
        fy = self.y + self.cg2frontaxle * np.sin(self.yaw + self.halfpi)

        dx = [fx - icx for icx in self.cx] # Find the x-axis of the front axle relative to the path
        dy = [fy - icy for icy in self.cy] # Find the y-axis of the front axle relative to the path

        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_idx = np.argmin(d) # Find the shortest distance in the array

        # Cross track error, project RMS error onto the front axle vector
        front_axle_vec = [-np.cos(self.yaw + np.pi), -np.sin(self.yaw + np.pi)]
        self.crosstrack_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        # Heading error
        self.heading_error = self.normalise_angle(self.cyaw[target_idx] - self.yaw - self.halfpi)
        self.target_idx = target_idx

        # Yaw rate discrepancy
        # yawrate_error = self.trajectory_yawrate_calc() - self.yawrate
        self.yawrate_error = 0.0
    
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

            if (n >= 0) and ((n + 1) < len(cx)):

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

        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)        
        return dist

    # Stanley controller determines the appropriate steering angle
    def stanley_control(self):
        
        crosstrack_term = np.arctan2((self.k * self.crosstrack_error), (self.ksoft + self.target_vel))
        heading_term = self.normalise_angle(self.heading_error)
        yawrate_term = self.kyaw * self.yawrate_error

        sigma_t = crosstrack_term + heading_term + yawrate_term

        # Constrains steering angle to the vehicle limits
        if sigma_t >= self.max_steer:
            sigma_t = self.max_steer
        elif sigma_t <= -self.max_steer:
            sigma_t = -self.max_steer
        else:
            pass

        self.set_vehicle_command(self.target_vel, sigma_t)

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
    n = 0
    # Initialise the node
    rospy.init_node('path_tracker')

    rospy.wait_for_message('/ngeeann_av/state2D', State2D)
    rospy.wait_for_message('/ngeeann_av/path', Path2D)

    # Initialise the class
    path_tracker = PathTracker()
    # Set update rate
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            if (path_tracker.cyaw != []):
                path_tracker.stanley_control()
            r.sleep()

            if n == 50:
                print("Current Tracking Error: {} m".format(path_tracker.crosstrack_error))
                print("Point {} of {} in current path".format(path_tracker.target_idx, len(path_tracker.cyaw)))
                n = 0
            else:
                n += 1


        except KeyboardInterrupt:
            print("\n")
            print("Execution time: {}".format(datetime.datetime.now() - begin_time))
            print("Shutting down ROS node...")

if __name__ == "__main__":
    main()
