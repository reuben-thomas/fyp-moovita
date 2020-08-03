#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose2D, Quaternion, PoseStamped
from nav_msgs.msg import Path
from ngeeann_av_nav.msg import Path2D, State2D

class GlobalPathPlanner:

    def __init__(self):

        ''' Class constructor to initialise the class '''

        # Initialise publisher(s)
        self.goals_pub = rospy.Publisher('/ngeeann_av/goals', Path2D, queue_size=10)
        self.goals_viz_pub = rospy.Publisher('/ngeeann_av/viz_goals', Path, queue_size=10)

        # Initialise suscriber(s)
        #self.targets_sub = rospy.Subscriber('/ngeeann_av/current_target', Pose2D, self.target_check_cb, queue_size=10)

        # Load parameters
        try:
            self.global_planner_params = rospy.get_param("/global_path_planner")
            self.frequency = self.global_planner_params["update_frequency"]
            self.givenwp = self.global_planner_params["given_number_of_waypoints"]
            self.tolerance = self.global_planner_params["target_tolerance"]

            if self.givenwp < 2:
                self.givenwp == 2
            
            else:
                pass

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Import waypoints.csv into class variables ax and ay
        self.ax = [103, 101, 99, 96, 91, 86, 80, 74, 66, 58]
        self.ay = [0, 10, 19, 29, 39, 48, 56, 64, 72, 79]

        # Class variables to use whenever within the class when necessary
        self.alive = False

        self.lowerbound = 0
        self.upperbound = self.lowerbound + (self.givenwp)

        self.lowerindex = 0
        self.upperindex = self.lowerindex + (self.givenwp - 1)

        self.ax_pub = self.ax[self.lowerbound : self.upperbound]
        self.ay_pub = self.ay[self.lowerbound : self.upperbound]

        self.current_target = None

        self.points = 1
        self.total_goals = 0

    def almost_reached(self):

        ''' Tells the node when to compute and publish the waypoints to the Local Path Planner '''
        
        self.set_waypoints(False)

        print("\nVehicle has almost reached waypoint {}".format(self.points))
        self.points += 1

    def set_waypoints(self, first):

        ''' Set the waypoints that are to be published, given the vehicle's position ''' 

        if first == True:

            # If there is not enough waypoints to publish
            if  self.givenwp >= len(self.ax):
                
                # Publish entire array
                self.publish_goals(self.ax, self.ay)

            else:
                self.publish_goals(self.ax_pub, self.ay_pub)

        else:
            if  self.givenwp > (len(self.ax) - self.lowerindex): # If the number of waypoints to give is more than the number of waypoints left
                self.upperbound = self.lowerbound + (len(self.ax) - self.lowerbound) # New upper index

                self.ax_pub = self.ax[self.lowerbound : self.upperbound]
                self.ay_pub = self.ay[self.lowerbound : self.upperbound]


            else: # If the number of waypoints to give is less or equal to the number of waypoints left.
                self.lowerbound += self.givenwp
                self.upperbound += self.givenwp
                self.lowerindex += self.givenwp
                self.upperindex += self.givenwp
                
                self.ax_pub = self.ax[self.lowerbound : self.upperbound]
                self.ay_pub = self.ay[self.lowerbound : self.upperbound]

            print("ax_pub:{}\nay_pub{}".format(self.ax_pub, self.ay_pub))
            self.publish_goals(self.ax_pub, self.ay_pub)

    def publish_goals(self, ax, ay):

        ''' Publishes an array of waypoints for the Local Path Planner '''
        goals = Path2D()

        viz_goals = Path()
        viz_goals.header.frame_id = "map"
        viz_goals.header.stamp = rospy.Time.now()

        goals_per_publish = 0
        self.total_goals += 1

        for i in range(0, self.givenwp):
            # Appending to Target Goals
            goal = Pose2D()
            goal.x = ax[i]
            goal.y = ay[i]
            goals.poses.append(goal)
            
            # Appending to Visualization Path
            vpose = PoseStamped()
            vpose.header.frame_id = "map"
            vpose.header.seq = i
            vpose.header.stamp = rospy.Time.now()
            vpose.pose.position.x = ax[i]
            vpose.pose.position.y = ay[i]
            vpose.pose.position.z = 0.0
            viz_goals.poses.append(vpose)
            
            goals_per_publish += 1

        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)

        print("\n")
        print("Total goals published: {}".format(self.total_goals))
        print("Goals per publish:{}".format(goals_per_publish))
        print("\nPublished goals: \n{}".format(goals))
        
def main():

    ''' Main function to initialise the class and node. '''
    
    # Initialise the class
    global_planner = GlobalPathPlanner()

    # Initialise the node
    rospy.init_node('global_planner')

    # Set update rate
    r = rospy.Rate(global_planner.frequency)

    # Publishes the first goal
    global_planner.set_waypoints(True)

    print_alive = True

    rospy.sleep(2)
    global_planner.almost_reached()

    while not rospy.is_shutdown():
        try:
            r.sleep()

        except KeyboardInterrupt:
            print("\n")
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()