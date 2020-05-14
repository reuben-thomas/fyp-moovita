#!/usr/bin/env python

"""ackermann_controller

Control the wheels of a vehicle with Ackermann steering.

Subscribed Topics:
    ackermann_cmd (ackermann_msgs/AckermannDrive)
        Ackermann command. It contains the vehicle's desired speed and steering
        angle.

Published Topics:
    <left steering controller name>/command (std_msgs/Float64)
        Command for the left steering controller.
    <right steering controller name>/command (std_msgs/Float64)
        Command for the right steering controller.
    <left front axle controller name>/command (std_msgs/Float64)
        Command for the left front axle controller.
    <right front axle controller name>/command (std_msgs/Float64)
        Command for the right front axle controller.
    <left rear axle controller name>/command (std_msgs/Float64)
        Command for the left rear axle controller.
    <right rear axle controller name>/command (std_msgs/Float64)
        Command for the right rear axle controller.
    <shock absorber controller name>/command (std_msgs/Float64)
        One of these topics exists for each shock absorber. They are latched
        topics.

Services Called:
    controller_manager/list_controllers (controller_manager_msgs/
                                         ListControllers)
        List the states of the controllers.

Parameters:
    ~left_front_wheel/steering_link_name (string, default: left_steering_link)
    ~right_front_wheel/steering_link_name (string,
                                           default: right_steering_link)
        Names of links that have origins coincident with the origins of the
        left and right steering joints, respectively. The steering links are
        used to compute the distance between the steering joints, as well as
        the vehicle's wheelbase.

    ~left_front_wheel/steering_controller_name (string, default:
                                                left_steering_controller)
    ~right_front_wheel/steering_controller_name (string, default:
                                                 right_steering_controller)
        Steering controller names.

    ~left_rear_wheel/link_name (string, default: left_wheel)
    ~right_rear_wheel/link_name (string, default: right_wheel)
        Names of links that have origins coincident with the centers of the
        left and right wheels, respectively. The rear wheel links are used to
        compute the vehicle's wheelbase.

    ~left_front_wheel/axle_controller_name (string)
    ~right_front_wheel/axle_controller_name
    ~left_rear_wheel/axle_controller_name
    ~right_rear_wheel/axle_controller_name
        Axle controller names. If no controller name is specified for an axle,
        that axle will not have a controller. This allows the control of
        front-wheel, rear-wheel, and four-wheel drive vehicles.

    ~left_front_wheel/diameter (double, default: 1.0)
    ~right_front_wheel/diameter
    ~left_rear_wheel/diameter
    ~right_rear_wheel/diameter
        Wheel diameters. Each diameter must be greater than zero. Unit: meter.

    ~shock_absorbers (sequence of mappings, default: empty)
        Zero or more shock absorbers.

        Key-Value Pairs:

        controller_name (string)
            Controller name.
        equilibrium_position (double, default: 0.0)
            Equilibrium position. Unit: meter.

    ~cmd_timeout (double, default: 0.5)
        If ~cmd_timeout is greater than zero and this node does not receive a
        command for more than ~cmd_timeout seconds, vehicle motion is paused
        until a command is received. If ~cmd_timeout is less than or equal to
        zero, the command timeout is disabled.
    ~publishing_frequency (double, default: 30.0)
        Joint command publishing frequency. It must be greater than zero.
        Unit: hertz.

Required tf Transforms:
    <~left_front_wheel/steering_link_name> to <~right_rear_wheel/link_name>
        Specifies the position of the left front wheel's steering link in the
        right rear wheel's frame.
    <~right_front_wheel/steering_link_name> to <~right_rear_wheel/link_name>
        Specifies the position of the right front wheel's steering link in the
        right rear wheel's frame.
    <~left_rear_wheel/link_name> to <~right_rear_wheel/link_name>
        Specifies the position of the left rear wheel in the right rear
        wheel's frame.

Copyright (c) 2013-2015 Wunderkammer Laboratory

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import math
import numpy
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers


class _AckermannCtrlr(object):
    """Ackermann controller

    An object of class _AckermannCtrlr is a node that controls the wheels of a
    vehicle with Ackermann steering.
    """

    def __init__(self):
        """Initialize this _AckermannCtrlr."""

        rospy.init_node("ackermann_controller")

        # Parameters

        # Wheels
        (left_steer_link_name, left_steer_ctrlr_name,
         left_front_axle_ctrlr_name, self._left_front_inv_circ) = \
                self._get_front_wheel_params("left")

        (right_steer_link_name, right_steer_ctrlr_name,
         right_front_axle_ctrlr_name, self._right_front_inv_circ) = \
                self._get_front_wheel_params("right")

        (left_rear_link_name, left_rear_axle_ctrlr_name,
         self._left_rear_inv_circ) = \
                self._get_rear_wheel_params("left")

        (self._right_rear_link_name, right_rear_axle_ctrlr_name,
         self._right_rear_inv_circ) = \
                self._get_rear_wheel_params("right")

        list_ctrlrs = rospy.ServiceProxy("controller_manager/list_controllers",
                                         ListControllers)
        list_ctrlrs.wait_for_service()

        # Shock absorbers
        shock_param_list = rospy.get_param("~shock_absorbers", [])
        self._shock_pubs = []
        try:
            for shock_params in shock_param_list:
                try:
                    ctrlr_name = shock_params["controller_name"]
                    try:
                        eq_pos = shock_params["equilibrium_position"]
                    except:
                        eq_pos = self._DEF_EQ_POS
                    eq_pos = float(eq_pos)
                except:
                    rospy.logwarn("An invalid parameter was specified for a "
                                  "shock absorber. The shock absorber will "
                                  "not be used.")
                    continue

                pub = rospy.Publisher(ctrlr_name + "/command", Float64,
                                      latch=True, queue_size=1)
                _wait_for_ctrlr(list_ctrlrs, ctrlr_name)
                pub.publish(eq_pos)
                self._shock_pubs.append(pub)
        except:
            rospy.logwarn("The specified list of shock absorbers is invalid. "
                          "No shock absorbers will be used.")

        # Command timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout",
                                                      self._DEF_CMD_TIMEOUT))
        except:
            rospy.logwarn("The specified command timeout value is invalid. "
                          "The default timeout value will be used instead.")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT

        # Publishing frequency
        try:
            pub_freq = float(rospy.get_param("~publishing_frequency",
                                             self._DEF_PUB_FREQ))
            if pub_freq <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified publishing frequency is invalid. "
                          "The default frequency will be used instead.")
            pub_freq = self._DEF_PUB_FREQ
        self._sleep_timer = rospy.Rate(pub_freq)

        # _last_cmd_time is the time at which the most recent Ackermann
        # driving command was received.
        self._last_cmd_time = rospy.get_time()

        # _ackermann_cmd_lock is used to control access to _steer_ang,
        # _steer_ang_vel, _speed, and _accel.
        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0      # Steering angle
        self._steer_ang_vel = 0.0  # Steering angle velocity
        self._speed = 0.0
        self._accel = 0.0          # Acceleration

        self._last_steer_ang = 0.0  # Last steering angle
        self._theta_left = 0.0      # Left steering joint angle
        self._theta_right = 0.0     # Right steering joint angle

        self._last_speed = 0.0
        self._last_accel_limit = 0.0  # Last acceleration limit
        # Axle angular velocities
        self._left_front_ang_vel = 0.0
        self._right_front_ang_vel = 0.0
        self._left_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0

        # _joint_dist_div_2 is the distance between the steering joints,
        # divided by two.
        tfl = tf.TransformListener()
        ls_pos = self._get_link_pos(tfl, left_steer_link_name)
        rs_pos = self._get_link_pos(tfl, right_steer_link_name)
        self._joint_dist_div_2 = numpy.linalg.norm(ls_pos - rs_pos) / 2
        lrw_pos = self._get_link_pos(tfl, left_rear_link_name)
        rrw_pos = numpy.array([0.0] * 3)
        front_cent_pos = (ls_pos + rs_pos) / 2     # Front center position
        rear_cent_pos = (lrw_pos + rrw_pos) / 2    # Rear center position
        self._wheelbase = numpy.linalg.norm(front_cent_pos - rear_cent_pos)
        self._inv_wheelbase = 1 / self._wheelbase  # Inverse of _wheelbase
        self._wheelbase_sqr = self._wheelbase ** 2

        # Publishers and subscribers

        self._left_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, left_steer_ctrlr_name)
        self._right_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, right_steer_ctrlr_name)

        self._left_front_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, left_front_axle_ctrlr_name)
        self._right_front_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, right_front_axle_ctrlr_name)
        self._left_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, left_rear_axle_ctrlr_name)
        self._right_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(list_ctrlrs, right_rear_axle_ctrlr_name)

        self._ackermann_cmd_sub = \
            rospy.Subscriber("ackermann_cmd", AckermannDrive,
                             self.ackermann_cmd_cb, queue_size=1)

    def spin(self):
        """Control the vehicle."""

        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            if (self._cmd_timeout > 0.0 and
                t - self._last_cmd_time > self._cmd_timeout):
                # Too much time has elapsed since the last command. Stop the
                # vehicle.
                steer_ang_changed, center_y = \
                    self._ctrl_steering(self._last_steer_ang, 0.0, 0.001)
                self._ctrl_axles(0.0, 0.0, 0.0, steer_ang_changed, center_y)
            elif delta_t > 0.0:
                with self._ackermann_cmd_lock:
                    steer_ang = self._steer_ang
                    steer_ang_vel = self._steer_ang_vel
                    speed = self._speed
                    accel = self._accel
                steer_ang_changed, center_y = \
                    self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)
                self._ctrl_axles(speed, accel, delta_t, steer_ang_changed,
                                 center_y)

            # Publish the steering and axle joint commands.
            self._left_steer_cmd_pub.publish(self._theta_left)
            self._right_steer_cmd_pub.publish(self._theta_right)
            if self._left_front_axle_cmd_pub:
                self._left_front_axle_cmd_pub.publish(self._left_front_ang_vel)
            if self._right_front_axle_cmd_pub:
                self._right_front_axle_cmd_pub.\
                    publish(self._right_front_ang_vel)
            if self._left_rear_axle_cmd_pub:
                self._left_rear_axle_cmd_pub.publish(self._left_rear_ang_vel)
            if self._right_rear_axle_cmd_pub:
                self._right_rear_axle_cmd_pub.publish(self._right_rear_ang_vel)

            self._sleep_timer.sleep()

    def ackermann_cmd_cb(self, ackermann_cmd):
        """Ackermann driving command callback

        :Parameters:
          ackermann_cmd : ackermann_msgs.msg.AckermannDrive
            Ackermann driving command.
        """
        self._last_cmd_time = rospy.get_time()
        with self._ackermann_cmd_lock:
            self._steer_ang = ackermann_cmd.steering_angle
            self._steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self._speed = ackermann_cmd.speed
            self._accel = ackermann_cmd.acceleration

    def _get_front_wheel_params(self, side):
        # Get front wheel parameters. Return a tuple containing the steering
        # link name, steering controller name, axle controller name (or None),
        # and inverse of the circumference.

        prefix = "~" + side + "_front_wheel/"
        steer_link_name = rospy.get_param(prefix + "steering_link_name",
                                          side + "_steering_link")
        steer_ctrlr_name = rospy.get_param(prefix + "steering_controller_name",
                                           side + "_steering_controller")
        axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
        return steer_link_name, steer_ctrlr_name, axle_ctrlr_name, inv_circ

    def _get_rear_wheel_params(self, side):
        # Get rear wheel parameters. Return a tuple containing the link name,
        # axle controller name, and inverse of the circumference.

        prefix = "~" + side + "_rear_wheel/"
        link_name = rospy.get_param(prefix + "link_name", side + "_wheel")
        axle_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
        return link_name, axle_ctrlr_name, inv_circ

    def _get_common_wheel_params(self, prefix):
        # Get parameters used by the front and rear wheels. Return a tuple
        # containing the axle controller name (or None) and the inverse of the
        # circumference.

        axle_ctrlr_name = rospy.get_param(prefix + "axle_controller_name",
                                          None)

        try:
            dia = float(rospy.get_param(prefix + "diameter",
                                        self._DEF_WHEEL_DIA))
            if dia <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified wheel diameter is invalid. "
                          "The default diameter will be used instead.")
            dia = self._DEF_WHEEL_DIA

        return axle_ctrlr_name, 1 / (pi * dia)

    def _get_link_pos(self, tfl, link):
        # Return the position of the specified link, relative to the right
        # rear wheel link.

        while True:
            try:
                trans, not_used = \
                    tfl.lookupTransform(self._right_rear_link_name, link,
                                        rospy.Time(0))
                return numpy.array(trans)
            except:
                pass

    def _ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        # Control the steering joints.

        # Compute theta, the virtual front wheel's desired steering angle.
        if steer_ang_vel_limit > 0.0:
            # Limit the steering velocity.
            ang_vel = (steer_ang - self._last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit,
                           min(ang_vel, steer_ang_vel_limit))
            theta = self._last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        # Compute the desired steering angles for the left and right front
        # wheels.
        center_y = self._wheelbase * math.tan((pi / 2) - theta)
        steer_ang_changed = theta != self._last_steer_ang
        if steer_ang_changed:
            self._last_steer_ang = theta
            self._theta_left = \
                _get_steer_ang(math.atan(self._inv_wheelbase *
                                         (center_y - self._joint_dist_div_2)))
            self._theta_right = \
                _get_steer_ang(math.atan(self._inv_wheelbase *
                                         (center_y + self._joint_dist_div_2)))

        return steer_ang_changed, center_y

    def _ctrl_axles(self, speed, accel_limit, delta_t, steer_ang_changed,
                    center_y):
        # Control the axle joints.

        # Compute veh_speed, the vehicle's desired speed.
        if accel_limit > 0.0:
            # Limit the vehicle's acceleration.
            self._last_accel_limit = accel_limit
            accel = (speed - self._last_speed) / delta_t
            accel = max(-accel_limit, min(accel, accel_limit))
            veh_speed = self._last_speed + accel * delta_t
        else:
            self._last_accel_limit = accel_limit
            veh_speed = speed

        # Compute the desired angular velocities of the wheels.
        if veh_speed != self._last_speed or steer_ang_changed:
            self._last_speed = veh_speed
            left_dist = center_y - self._joint_dist_div_2
            right_dist = center_y + self._joint_dist_div_2

            # Front
            gain = (2 * pi) * veh_speed / abs(center_y)
            r = math.sqrt(left_dist ** 2 + self._wheelbase_sqr)
            self._left_front_ang_vel = gain * r * self._left_front_inv_circ
            r = math.sqrt(right_dist ** 2 + self._wheelbase_sqr)
            self._right_front_ang_vel = gain * r * self._right_front_inv_circ
            # Rear
            gain = (2 * pi) * veh_speed / center_y
            self._left_rear_ang_vel = \
                gain * left_dist * self._left_rear_inv_circ
            self._right_rear_ang_vel = \
                gain * right_dist * self._right_rear_inv_circ

    _DEF_WHEEL_DIA = 1.0    # Default wheel diameter. Unit: meter.
    _DEF_EQ_POS = 0.0       # Default equilibrium position. Unit: meter.
    _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
    _DEF_PUB_FREQ = 30.0    # Default publishing frequency. Unit: hertz.
# end _AckermannCtrlr


def _wait_for_ctrlr(list_ctrlrs, ctrlr_name):
    # Wait for the specified controller to be in the "running" state.
    # Commands can be lost if they are published before their controller is
    # running, even if a latched publisher is used.

    while True:
        response = list_ctrlrs()
        for ctrlr in response.controller:
            if ctrlr.name == ctrlr_name:
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)
                break


def _create_axle_cmd_pub(list_ctrlrs, axle_ctrlr_name):
    # Create an axle command publisher.
    if not axle_ctrlr_name:
        return None
    return _create_cmd_pub(list_ctrlrs, axle_ctrlr_name)


def _create_cmd_pub(list_ctrlrs, ctrlr_name):
    # Create a command publisher.
    _wait_for_ctrlr(list_ctrlrs, ctrlr_name)
    return rospy.Publisher(ctrlr_name + "/command", Float64, queue_size=1)


def _get_steer_ang(phi):
    # Return the desired steering angle for a front wheel.
    if phi >= 0.0:
        return (pi / 2) - phi
    return (-pi / 2) - phi


# main
if __name__ == "__main__":
    ctrlr = _AckermannCtrlr()
    ctrlr.spin()
