#!/usr/bin/env python

import threading, rospy, tf
import numpy as np

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers

class AckermannController:

    def __init__(self):
        pass
        
def main():

    # Initialise the class
    controller = _AckermannController()

    # Initialise the node
    rospy.init_node("ackermann_controller")

    controller.spin()

if __name__ == "__main__":
    main()