#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, Twist, Pose, Vector3
from math import cos, sin, pi

import csv
import os


class CircularPathPublisher:
    def __init__(self):
        rospy.init_node('circular_path_publisher')

        self.radius = 1
        self.freq = 60
        T = 40
        self.angular_velocity = 2 * pi / T

        self.start_time = None

        # Create a CSV file to store the data
        folder_path = '/root/data/'

        # Create the file in the specified folder
        file_path = os.path.join(folder_path, 'circular_path_data.csv')
        csv_file = open(file_path, 'w')

        self.csv_writer = csv.writer(csv_file)
        self.csv_writer.writerow(['Time', 'X', 'Y', 'VX', 'VY'])

        self.publisher = rospy.Publisher('path_pose', 
                                         Odometry, 
                                         queue_size=10)

        self.subscription = rospy.Subscriber('emergency_flag', 
                                             Bool, 
                                             self.emergency_button_callback, 
                                             queue_size=10)
        
        rospy.Timer(rospy.Duration(1/self.freq), self.publish_odometry)

        rospy.loginfo('Path publisher node started')


    def publish_odometry(self, event):
        if self.start_time is None:
            self.start_time = rospy.Time.now()

        # Get the current timestamp
        current_time = rospy.Time.now()

        # Calculate elapsed time
        elapsed_time = (current_time - self.start_time).to_sec()

        x = self.radius * cos(self.angular_velocity * elapsed_time)
        y = self.radius * sin(self.angular_velocity * elapsed_time)
        vx = -self.radius * self.angular_velocity * sin(self.angular_velocity * elapsed_time)
        vy = self.radius * self.angular_velocity * cos(self.angular_velocity * elapsed_time)

        # # Write the data to the CSV file
        self.csv_writer.writerow([current_time.to_sec(), x, y, vx, vy])

        # Create the Odometry message
        odometry = Odometry()
        odometry.header.stamp = current_time
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion())
        odometry.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odometry.twist.twist.angular = Vector3()

        self.publisher.publish(odometry)


    def emergency_button_callback(self, msg):
        if msg.data:
            rospy.loginfo('Path publisher node stopping by Emergency')
            rospy.signal_shutdown('Emergency stop')


if __name__ == '__main__':
    circular_path_publisher = CircularPathPublisher()
    rospy.spin()