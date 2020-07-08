#!/usr/bin/env python

import rospy, cubic_spline_planner
from geometry_msgs.msg import Pose2D
from ackermann_msgs.msg import AckermannDrive
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Path

target_vel = 5.0 # Target Velocity 
k = 1.0 # Control Gain
ksoft = 1.0 # Softening gain to ensure a non-zero denominator
max_steer = 0.95  # Max steering angles in radians
cog2frontaxle = 1.483 # Distance from the vehicle's centre of gravity to its front axle
halfpi = np.pi / 2


def main():
    path_tracker = rospy.Subscriber('/ngeeann_av/path', Path, queue_size=3)
    localisation = rospy.Subscriber('/ngeeann_av/state2D', Pose2D, localisation)

    rospy.init_node('path_tracker')
    r = rospy.Rate(30) # Set update rate, default to 30

    ax = [100.0, 100.0, 96.0, 90.0, 0.0]
    ay = [18.3, 31.0, 43.0, 47.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    last_idx = len(cx) - 1

    target_idx, _ = calc_target_index(cx, cy)

    while not rospy.is_shutdown():
        try:
            di, target_idx = stanley_control(cx, cy, cyaw, target_idx)

            set_vehicle_command(target_vel, di)
            r.sleep()

        except rospy.ServiceException as e:
            rospy.loginfo("Path tracking node failed:  {0}".format(e))

def localisation(coordinates):
    x = coordinates.x
    y = coordinates.y
    theta = coordinates.theta

#Sets vehicle command
def set_vehicle_command (velocity, steering_angle):
    drive = AckermannDrive()
    drive.speed = velocity
    drive.acceleration = 1.0
    drive.steering_angle = steering_angle
    drive.steering_angle_velocity = 0.0
    navigation.publish(drive)

def stanley_control(cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    #recieves yaw
    yaw = get_yaw_rad()

    # theta_e corrects the heading error
    theta_e = normalize_angle((cyaw[current_target_idx]-halfpi) - yaw)
    print('Heading error = ' + str(cyaw[current_target_idx]) + ' - ' + str(yaw) + ' = ' + str(theta_e))

    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, ksoft + target_vel)
    
    # Steering control
    delta = theta_e + theta_d

    if delta >= 0.95:
        delta = 0.95
    elif delta <= -0.95:
        delta = -0.95

    return delta, current_target_idx

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

def calc_target_index(cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """

    #recieves yaw
    yaw = get_yaw_rad()

    # Calc front axle position
    fx = state.pose.position.x + cog2frontaxle * np.cos((yaw)+halfpi)
    fy = state.pose.position.y + cog2frontaxle * np.sin((yaw)+halfpi)
    print('front axle (fx, fy): (' + str(fx) + ', ' + str(fy) + ')')

    # Search nearest point index
    dx = [fx - icx for icx in cx] # Find the x-axis of the front axle relative to the path
    dy = [fy - icy for icy in cy] # Find the y-axis of the front axle relative to the path
    d = np.hypot(dx, dy) # Find the distance from the front axle to the path
    target_idx = np.argmin(d) # Find the shortest distance in the array

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos((yaw+halfpi)+ halfpi), -np.sin((yaw+halfpi)+ halfpi)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    print('Target (x,y): (' + str(cx[target_idx]) + ', ' + str(cy[target_idx]) + ')')
    print('Front axle error: ' + str(error_front_axle))

    return target_idx, error_front_axle

if __name__=="__main__":
    main()
