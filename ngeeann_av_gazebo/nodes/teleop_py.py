#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive
import sys

msg = AckermannDrive()

def values():
    print '(w for forward, a for left, s for reverse, d for right,k for turning back left,l for turning back right and . to exit)' + '\n'
    s = raw_input(':- ')
    if s[0] == 'w':
        msg.steering_angle = 0.0
        msg.steering_angle_velocity = 0.0
        msg.speed = 10.0
        msg.acceleration = 3.0
        msg.jerk = 0
    elif s[0] == 's':
        msg.steering_angle = 0.0
        msg.steering_angle_velocity = 0.0
        msg.speed = -10.0
        msg.acceleration = 3.0
        msg.jerk = 0
    elif s[0] == 'd':
        msg.steering_angle = 0.80
        msg.steering_angle_velocity = 0.0
        msg.speed = 5.0
        msg.acceleration = 1.0
        msg.jerk = 0
    elif s[0] == 'a':
        msg.steering_angle = -0.80
        msg.steering_angle_velocity = 0.0
        msg.speed = 5.0
        msg.acceleration = 1.0
        msg.jerk = 0
    elif s[0] == 'k':
        msg.steering_angle = -0.80
        msg.steering_angle_velocity = 0.0
        msg.speed = 5.0
        msg.acceleration = 1.0
        msg.jerk = 0
    elif s[0] == 'l':
        msg.steering_angle = -0.80
        msg.steering_angle_velocity = 0.0
        msg.speed = 5.0
        msg.acceleration = 1.0
        msg.jerk = 0
    elif s[0] == '.':
        msg.steering_angle = msg.steering_angle_velocity = msg.speed = msg.acceleration = msg.jerk = 0.0
        sys.exit()
    else:
        msg.steering_angle = msg.steering_angle_velocity = msg.speed = msg.acceleration = msg.jerk = 0.0
        print 'Wrong command entered \n'
    return msg

def keyboard():
    pub = rospy.Publisher('/ngeeann_av/ackermann_cmd',AckermannDrive, queue_size=1)
    rospy.init_node('teleop_py',anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        twist = values()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard()
    except rospy.ROSInterruptException:
        pass

