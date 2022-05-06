#!/usr/bin/env python3
# a template for the move_square exercise

import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

from sensor_msgs.msg import LaserScan

class NavigationNode:
    
    def LaserListener(self, ScanData :LaserScan):
        # obtain the orientation co-ords:
        self.Ranges = ScanData.ranges

    def __init__(self):
        node_name = "NavigationNode"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() to switch between turning and moving forwards...?
        self.turn = False

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("scan", LaserScan, self.LaserListener)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.Ranges = {}
        self.ObstacleDirection = 270
        
        # define a Twist instance, which can be used to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def print_stuff(self, a_message):
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        print(a_message)
        # you could use this to print the current velocity command:
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        # you could also print the current odometry to the terminal here, if you wanted to:
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")

    def main_loop(self):
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your 
            # robot. Add code here to make your robot move in a square of
            # dimensions 0.5x0.5m...
            
            self.ObstacleDirection = len(self.Ranges)
            # Find nearest obstacle from ranges
            for i in range(len(self.Ranges)):
                if(self.Ranges[i] < .5):
                    self.ObstacleDirection = i

            self.ObstacleDirection+=720
            self.ObstacleDirection%=360

            self.vel.linear.x = .1
            self.vel.angular.z = 0

            if self.ObstacleDirection == 0:
                self.vel.linear.x = .1
                self.vel.angular.z = 0
            elif self.ObstacleDirection > 330 or self.ObstacleDirection < 90:
                print(f" Wants to turn right")
                self.vel.linear.x = 0
                self.vel.angular.z = .4
            elif self.ObstacleDirection > 90 and self.ObstacleDirection < 210:
                print(f" Wants to turn left")
                self.vel.linear.x = 0
                self.vel.angular.z = -0.4
            
            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            print(f" i = {self.ObstacleDirection}, length = {len(self.Ranges)}")
            self.rate.sleep()

if __name__ == '__main__':
    NavNodeInstance = NavigationNode()
    try:
        NavNodeInstance.main_loop()
    except rospy.ROSInterruptException:
        pass