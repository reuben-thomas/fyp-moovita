#!/usr/bin/env python
import threading
import sys
import math
import rospy
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from ngeeann_av_nav.msg import Path2D, State2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
import sensor_msgs.point_cloud2 as pc2

class CruiseControl(object):
    