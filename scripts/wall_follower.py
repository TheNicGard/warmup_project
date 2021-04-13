#!/usr/bin/env python3
""" This script commands the Turtlebot3 robot to follow the wall. """
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import inf, pi
from statistics import median
import rospy
import time

class WallFollower(object):
    def __init__(self):
        rospy.init_node('wall_follower')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.get_scan)
        
        self.TURN_SPEED = 0.7
        self.DRIVE_SPEED = 0.5
        
        self.SAFE_DIST = 0.5
        self.SAFE_MARGIN = 0.1

        """ a dict containing three "rays," which point out from the robot. """
        self.rays = {
            "front": inf,
            "left": inf,
            "right": inf,
        }

    """
    get_scan: callback function for /scan topic subscriber to get the
    direction/distance of the closest object at three specific rays.
    """
    def get_scan(self, data):
        self.rays["left"] = data.ranges[90]
        self.rays["front"] = data.ranges[0]
        self.rays["right"] = data.ranges[270]
    
    def run(self):
        """ Main functionality """
        rospy.sleep(5.0)
        
        while not rospy.is_shutdown():
            if self.rays["front"] < self.SAFE_DIST:
                if self.rays["left"] < self.SAFE_DIST:
                    """ If the robot is in a corner, drive forward and turn right. """
                    twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, -self.TURN_SPEED))
                    self.publisher.publish(twist)
                else:
                    """ If the robot is against a wall, turn right. """
                    twist = Twist(angular=Vector3(0, 0, -self.TURN_SPEED))
                    self.publisher.publish(twist)
            else:
                if self.rays["left"] < self.SAFE_DIST:
                    """ Drive forward. """
                    forward_twist = Twist(linear=Vector3(self.DRIVE_SPEED, 0, 0))
                    self.publisher.publish(forward_twist)
                else:
                    """ If the robot isn't near the wall, turn right. """
                    twist = Twist(linear=Vector3(self.DRIVE_SPEED, 0, 0), angular=Vector3(0, 0, 0.2))
                    self.publisher.publish(twist)


if __name__ == "__main__":
    node = WallFollower()
    node.run()
