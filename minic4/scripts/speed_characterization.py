#!/usr/bin/env python3

import rospy  # Import the rospy module for ROS functionality

# Import necessary message types
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class SpeedCharacterization:
    def __init__(self):
        self.wl = 0.0  # Initialize left wheel speed to 0
        self.wr = 0.0  # Initialize right wheel speed to 0

        # Create a publisher to send velocity commands to the robot
        self.pub = rospy.Publisher('/puzzlebot_1/base_controller/cmd_vel', Twist, queue_size=10)

        # Create publishers to publish linear and angular velocities
        self.pub_v = rospy.Publisher('/puzzlebot_1/v', Float32, queue_size=1)
        self.pub_w = rospy.Publisher('/puzzlebot_1/w', Float32, queue_size=1)

        # Create subscribers to receive left and right wheel speeds
        self.sub_wl = rospy.Subscriber('/puzzlebot_1/wl', Float32, self.wl_callback)
        self.sub_wr = rospy.Subscriber('/puzzlebot_1/wr', Float32, self.wr_callback)

        # Open text files to store linear and angular velocity data
        self.txt_file = open('/home/magotronico/Documents/output.txt', 'w')
        self.txt_file2 = open('/home/magotronico/Documents/output2.txt', 'w')

    def wl_callback(self, msg):
        # Callback function to update the left wheel speed
        self.wl = msg.data

    def wr_callback(self, msg):
        # Callback function to update the right wheel speed
        self.wr = msg.data

    def calculate_vw(self):
        # Calculate linear and angular velocities based on wheel speeds
        v = 0.05 * (self.wl + self.wr) / 2  # Linear velocity
        w = 0.05 * (self.wr - self.wl) / 0.19  # Angular velocity
        return v, w

    def test(self):
        # Initialize the ROS node
        rospy.init_node('speed_characterization', anonymous=True)
        rate = rospy.Rate(10)  # Set the loop rate to 10 Hz

        for i in range(101):  # Loop from 0 to 1 with a resolution of 0.01
            msg = Twist()  # Create a Twist message
            msg.linear.x = i / 100.0  # Set the linear velocity
            self.pub.publish(msg)  # Publish the velocity command

            rospy.sleep(0.1)  # Sleep for a while before the next iteration

            v, w = self.calculate_vw()  # Calculate linear and angular velocities
            self.pub_v.publish(v)  # Publish the linear velocity
            self.pub_w.publish(w)  # Publish the angular velocity

            rospy.loginfo("v: %s, w: %s", v, w)  # Log the velocities
            self.txt_file.write(f"{v}\n")  # Write the linear velocity to a file

            rate.sleep()  # Sleep to maintain the desired loop rate

    def test2(self):
        # Initialize the ROS node
        rospy.init_node('speed_characterization', anonymous=True)
        rate = rospy.Rate(10)  # Set the loop rate to 10 Hz

        for i in range(1001):  # Loop from 0 to 10 with a resolution of 0.01
            msg = Twist()  # Create a Twist message
            msg.angular.z = i / 100.0  # Set the angular velocity
            self.pub.publish(msg)  # Publish the velocity command

            rospy.sleep(0.1)  # Sleep for a while before the next iteration

            v, w = self.calculate_vw()  # Calculate linear and angular velocities
            self.pub_v.publish(v)  # Publish the linear velocity
            self.pub_w.publish(w)  # Publish the angular velocity

            rospy.loginfo("v: %s, w: %s", v, w)  # Log the velocities
            self.txt_file2.write(f"{w}\n")  # Write the angular velocity to a file

            rate.sleep()  # Sleep to maintain the desired loop rate

if __name__ == '__main__':
    try:
        sc = SpeedCharacterization()  # Create an instance of the SpeedCharacterization class
        sc.test()
        sc.test2() 
    except rospy.ROSInterruptException:
        pass
    finally:
        sc.txt_file.close()  # Close the linear velocity file
        sc.txt_file2.close()  # Close the angular velocity file