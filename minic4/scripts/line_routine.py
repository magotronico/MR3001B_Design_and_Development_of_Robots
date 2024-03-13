#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_straight():
    # Initialize the ROS node
    rospy.init_node('move_straight', anonymous=True)

    # Create a publisher to send Twist messages to the robot
    publisher = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=10)

    # Create a Twist message instance
    twist = Twist()

    # Set the linear and angular velocities to move straight
    twist.linear.x = 0.20  # Forward speed in m/s
    twist.angular.z = 0.0  # No rotation

    rate = rospy.Rate(10)  # Set the rate to 10 Hz

    while not rospy.is_shutdown():
        # Publish the Twist message to move the robot
        publisher.publish(twist)
        rate.sleep()  # Sleep to maintain the desired rate

if __name__ == '__main__':
    try:
        move_straight()
    except rospy.ROSInterruptException:
        pass