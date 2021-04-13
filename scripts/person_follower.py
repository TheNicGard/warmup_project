#!/usr/bin/env python3
""" This script commands the Turtlebot3 robot to follow the cloest object. """
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import inf, pi
from statistics import median
import rospy
import time

class PersonFollower(object):
    def __init__(self):
        rospy.init_node('person_follower')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.get_scan)
        
        self.prev_dist = []
        self.prev_angle = []
        self.distance_to_closest = inf
        self.angle_to_closest = 0
        
        self.TURN_SPEED = 1.2
        self.DRIVE_SPEED = 1
        
        self.SAFE_DIST = 0.5
        self.SAFE_MARGIN = 0.05
        # self.DEADZONE = pi / 36 # approximately 5 degrees
        self.DEADZONE = pi / 18 # approximately 20 degrees

    """
    get_scan: subscribe the the /scan topic to get the direction/distance of the
    closest object.
    """
    def get_scan(self, data):
        min_distance = inf
        angle_to_min = 0

        """
        Iterate over the scanner values to find the scanner direction with the
        lowest distance, and save both of these as the minimum distance/angle.
        """
        for i, dist in enumerate(data.ranges):
            if dist < min_distance:
                min_distance = dist
                angle_to_min = i

        """ Average the last 10 readings to reduce noise. """
        self.prev_dist.append(min_distance)
        self.prev_angle.append(angle_to_min)
        
        if len(self.prev_dist) > 10:
            self.prev_dist.pop(0)
        if len(self.prev_angle) > 10:
            self.prev_angle.pop(0)
            
        self.distance_to_closest = median(self.prev_dist)
        self.angle_to_closest = median(self.prev_angle)

    """
    get_angle: convert the scanner direction (a value from 0 to 359 CCW, roughly
    equivalent to a direction in degrees) to a value in radians from -pi to pi.
    """
    def get_angle(self):
        if self.angle_to_closest <= 180:
            """ Maps <0 to 180> in degrees to <0 to pi> in radians. """
            return (self.angle_to_closest / 180) * pi
        else:
            """
            this linear equation is a rough approximation of angles <181 to
            359> in degrees mapping to <-pi to 0> in radians.
            """
            m = pi / (359 - 181)
            b = (2 * pi) + 0.05
            return (m * self.angle_to_closest) - b

    def get_speed(self):
        dist_to_safe_zone = self.distance_to_closest - self.SAFE_DIST
        if dist_to_safe_zone > 0:
            return min(self.DRIVE_SPEED * (dist_to_safe_zone ** 3), self.DRIVE_SPEED)
        else:
            return max(10 * self.DRIVE_SPEED * (dist_to_safe_zone ** 3), 10 * -self.DRIVE_SPEED)
    
    def run(self):
        """ Main functionality """
        rospy.sleep(5.0)
        start_time = time.time()
        
        while not rospy.is_shutdown():
            """ Get the angle to the closest object. """
            angle = self.get_angle()

            if self.distance_to_closest < self.SAFE_DIST:
                forward_twist = Twist(linear=Vector3(self.get_speed(), 0, 0))
                print("driving " + str(self.get_speed()))
                self.publisher.publish(forward_twist)
            elif abs(angle) > self.DEADZONE:
                turn_twist = Twist(angular=Vector3(0, 0, self.TURN_SPEED * angle / pi))
                print("turning " + str(self.TURN_SPEED * angle / pi))
                self.publisher.publish(turn_twist)
            else:
                is_in_margin = abs(self.distance_to_closest - self.SAFE_DIST) < self.SAFE_MARGIN
                if self.distance_to_closest <= 3.6 and not is_in_margin:
                    forward_twist = Twist(linear=Vector3(self.get_speed(), 0, 0))
                    print("driving " + str(self.get_speed()))
                    self.publisher.publish(forward_twist)
                else:
                    self.publisher.publish(Twist())
                
            #rospy.sleep(0.05)

if __name__ == "__main__":
    node = PersonFollower()
    node.run()
