# warmup_project
## Behaviors
### Driving in a Square
* The goal of this behavior is to make the Turtlebot3 robot move forward and turn 90 degrees four times. My approach to accomplishing this was to write to commands (`move_forward` and `move_turn`) which would make the robot move a specified by distance/angle at a specified speed. These commands function by sending Twist messages to the `/cmd_vel` topic. Because Twist messages contain velocity information, each command also stops the robot momentarily at the end of the function in order to reset its linear and angular velocity to zero.
* The `__init__` function only initializes the node, initializes a publisher for the Twist message, and initializes a Twist message which can be used to reset the robot's velocity vectors to zero. Two functions, `move_forward` and `move_turn`, are used to send commands to the robot. Each works in roughly the same way: the function is called with a desired distance to move or angle to rotate to, and a speed for the linear/angular velocity. The start time is recorded, and while the current distance/angle of the robot is less than the distance/angle specific, a Twist message is sent containing the linear/angular velocity of the specified speed, and the current distance/angle is calculated based on the elapsed time. Once the desired distance/angle is reached, the stopping Twist message is sent, and `rospy.sleep()` is called so that the robot's velocity can be reset to zero. The `run` function calls `move_forward()` and `move_turn()` on a loop, such that four iterations of the loop cause the robot to move in a square.
* ![Figure 1: Turtlebot3, driving in a "square"](./driving_square.gif)
## Challenges
## Future Work
## Takeaways
