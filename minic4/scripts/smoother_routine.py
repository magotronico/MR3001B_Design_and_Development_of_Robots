#!/usr/bin/env python3

import math  # Import the math module for mathematical operations
import rospy  # Import the rospy module for ROS functionality

from geometry_msgs.msg import Twist  # Import the Twist message type

class SmootherMover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('SmootherMover', anonymous=True)

        # Create a publisher to send Twist messages to the robot
        self.publisher = rospy.Publisher('/smoother_cmd_vel', Twist, queue_size=10)

        # Create a Twist message instance
        self.twist = Twist()

        self.linear_velocity = 0.28  # Linear speed in m/s
        self.angular_velocity = 0.3  # Angular speed in rad/s
        self.side_length = 1  # Length of each side of the square in meters
        self.amount_of_sides = 4  # Number of sides of the square

        # Set the initial linear and angular velocities
        self.twist.linear.x = 0.0  # No forward speed initially
        self.twist.angular.z = 0.0  # No rotation initially

        # Set the rate for publishing messages
        self.rate = rospy.Rate(1)  # 1Hz

    def compute_angles(self):
        # Compute the angle to turn for each corner of the square
        return 2 * math.pi / self.amount_of_sides

    def move_forward(self):
        # Set the linear velocity to move forward
        self.twist.linear.x = self.linear_velocity
        self.twist.angular.z = 0.0

        # Calculate the duration to move for the side length
        duration = self.side_length / self.linear_velocity
        rospy.loginfo("Duration: %s", duration)

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.publisher.publish(self.twist)
            self.rate.sleep()

    def rotate(self, angle):
        # Set the angular velocity to rotate
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_velocity

        # Calculate the duration to rotate for the given angle
        duration = 2.2 * angle / self.angular_velocity  # 2.2 is a time compensator for open-loop control
        rospy.loginfo("Duration: %s", duration)

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.publisher.publish(self.twist)
            self.rate.sleep()

    def stop(self):
        # Stop the robot by setting linear and angular velocities to zero
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher.publish(self.twist)

    def start_routine(self):
        rospy.sleep(4)  # Wait for 4 seconds before starting

        angles = self.compute_angles()
        rospy.loginfo("Angles: %s", angles)

        for _ in range(self.amount_of_sides):
            self.move_forward()
            self.stop()
            rospy.sleep(1)  # Wait for 1 second

            self.rotate(angles)  # Rotate for the computed angle
            self.stop()
            rospy.sleep(1)  # Wait for 1 second

        self.stop()

if __name__ == '__main__':
    try:
        smoother_mover = SmootherMover()
        smoother_mover.start_routine()
    except rospy.ROSInterruptException:
        pass