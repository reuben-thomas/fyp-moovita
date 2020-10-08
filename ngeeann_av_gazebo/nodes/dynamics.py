import rospy, os
import numpy as np
import pandas as pd
import math

from gazebo_msgs.srv import GetModelState, SetModelState
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelState
from ngeeann_av_nav.msg import State2D

# non-linear lateral bicycle model
class Dynamics:

    def __init__(self):
        rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state')
        rospy.wait_for_service('/ngeeann_av/gazebo/set_model_state')
        self.get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)
        self.set_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/set_model_state', SetModelState)

        self.cmd_sub = rospy.Subscriber('/ngeeann_av/ackermann_cmd', AckermannDrive, self.cmd_cb)
        
        self.speed = None
        self.steering_angle = None

    def cmd_cb(self, msg):
        self.speed = msg.speed
        self.steering_angle = msg.steering_angle
    
    def print_input(self):
        print(self.speed)
        print(self.steering_angle)

    def set_state(self, id):
        state_msg = ModelState()
        state_msg.model_name = 'suv'
        state_msg.pose.position.x = self.ax[id]
        state_msg.pose.position.y = self.ay[id]
        state_msg.pose.position.z = 0.0
        
        heading = np.arctan2(state_msg.pose.position.y, state_msg.pose.position.x) + np.pi * 0.5

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = np.sin(heading / 2)
        state_msg.pose.orientation.w = np.cos(heading / 2)

        try:
            resp = self.set_model_srv( state_msg )
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


class NonLinearBicycleModel:

    self.max_steer = np.radians(30.0)  # [rad] max steering angle
    L = 2.9  # [m] Wheel base of vehicle
    dt = 0.1
    Lr = L / 2.0  # [m]
    Lf = L - Lr
    Cf = 1600.0 * 2.0  # N/rad
    Cr = 1700.0 * 2.0  # N/rad
    Iz = 2250.0  # kg/m2
    m = 1750.0  # kg

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.01, vy=0, omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.omega = omega
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

    def update(self, throttle, delta):
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        self.x = self.x + self.vx * math.cos(self.yaw) * dt - self.vy * math.sin(self.yaw) * dt
        self.y = self.y + self.vx * math.sin(self.yaw) * dt + self.vy * math.cos(self.yaw) * dt
        self.yaw = self.yaw + self.omega * dt
        self.yaw = normalize_angle(self.yaw)
        Ffy = -Cf * math.atan2(((self.vy + Lf * self.omega) / self.vx - delta), 1.0)
        Fry = -Cr * math.atan2((self.vy - Lr * self.omega) / self.vx, 1.0)
        R_x = self.c_r1 * self.vx
        F_aero = self.c_a * self.vx ** 2
        F_load = F_aero + R_x
        self.vx = self.vx + (throttle - Ffy * math.sin(delta) / m - F_load/m + self.vy * self.omega) * dt
        self.vy = self.vy + (Fry / m + Ffy * math.cos(delta) / m - self.vx * self.omega) * dt
        self.omega = self.omega + (Ffy * Lf * math.cos(delta) - Fry * Lr) / Iz * dt

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def main():

    ''' Main function to initialise the class and node. '''
    # Initialise the node
    rospy.init_node('dynamics')

    dynamics = Dynamics()

    # Set update rate
    r = rospy.Rate(30) 

    while not rospy.is_shutdown():
        try:
            r.sleep()
            dynamics.print_input

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()