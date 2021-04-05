#!/usr/bin/env python3
""" This script commands the Turtlebot3 robot to drive in a square path. """
from geometry_msgs.msg import Twist, Vector3
from math import pi
import rospy
import time

class DriveSquare(object):
    def __init__(self):
        rospy.init_node('drive_square')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.stop_twist = Twist()

    """
    move_forward: moves the Turtlebot3 robot forward by the specified distance
    at the specified speed.
    """
    def move_forward(self, desired_distance, speed):
        distance = 0
        start_time = time.time()
        forward_twist = Twist(linear=Vector3(speed, 0, 0))

        """
        Check if the desired distance has been reached, and calculate the
        current distance using the speed of the robot and the elapsed time.
        """
        while distance <= desired_distance:
            self.publisher.publish(forward_twist)
            distance = (time.time() - start_time) * speed

        """ Sets the robot's velocity to 0, and waits for 2 seconds. """
        self.publisher.publish(self.stop_twist)
        rospy.sleep(2.0)

    """
    move_turn: rotates the Turtlebot3 robot by the specified angle at the
    specified speed.
    """
    def move_turn(self, desired_angle, speed):
        angle = 0
        start_time = time.time()
        turn_twist = Twist(angular=Vector3(0, 0, speed))

        """
        Check if the desired angle has been reached, and calculate the
        current angle using the speed of the robot and the elapsed time.
        """
        while angle <= desired_angle:
            self.publisher.publish(turn_twist)
            angle = (time.time() - start_time) * speed

        """ Sets the robot's velocity to 0, and waits for 2 seconds. """
        self.publisher.publish(self.stop_twist)
        rospy.sleep(2.0)

    def run(self):
        """ Main functionality """
        rospy.sleep(4.0)
        while not rospy.is_shutdown():
            self.move_forward(1, 0.3)
            self.move_turn(pi / 2, 0.3)
            rospy.sleep(1.0)

if __name__ == "__main__":
    node = DriveSquare()
    node.run()
