#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
import matplotlib.pyplot as plt




rospy.init_node('navigation')
rospy.wait_for_service('/ngeeann_av/gazebo/get_model_state') 
get_model_srv = rospy.ServiceProxy('/ngeeann_av/gazebo/get_model_state', GetModelState)
navigation = rospy.Publisher('/ngeeann_av/ackermann_cmd',AckermannDrive, queue_size=1) 
r = rospy.Rate(30) #Set update rate, default to 30




#gets and prints model state
def get_vehicle_state ():
    state = get_model_srv('ngeeann_av', 'ground')
    """print 'Status.success', state.success
    print('\n\nPOSITION:')
    print('x: ' + str(state.pose.position.x))
    print('y: ' + str(state.pose.position.y))
    print('z: ' + str(state.pose.position.z))
    print('ORIENTATION:')
    print('x: ' + str(state.pose.orientation.x))
    print('y: ' + str(state.pose.orientation.y))
    print('z: ' + str(state.pose.orientation.z))"""



#Sets vehicle command
def set_vehicle_command (velocity, steering_angle):
    drive = AckermannDrive()
    drive.speed = velocity
    drive.acceleration = 0.0
    drive.steering_angle = steering_angle
    drive.steering_angle_velocity = 0.0
    navigation.publish(drive)






def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s






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






def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    get_vehicle_state()
    fx = get + L * np.cos(state.yaw)
    fy = state.pose.x + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle






def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx




#
if __name__=="__main__":
    #  target course
    ax = [100.0, 90.0, 70.0, 40.0, 0.0]
    ay = [0.0, 20.0, 40.0, 60.0, 80.0]

    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=0.1)
    


    while not rospy.is_shutdown():
        try:
            get_vehicle_state()
            set_vehicle_command(2.0, 0.0)
            r.sleep()
        except rospy.ServiceException as e:
            rospy.loginfo("Navigation node failed:  {0}".format(e))
