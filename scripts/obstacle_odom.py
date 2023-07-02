#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import time
import csv
import os
import math
import numpy as np


class OdomPublisher(object):
    def __init__(self):
        rospy.init_node('odom_publisher')

        self.odom = Odometry()

        self.timer = rospy.Timer(rospy.Duration(1/60), self.loop)
        
        folder_path = '/root/data/'

        # Create the file in the specified folder
        file_path = os.path.join(folder_path, 'object_odom.csv')
        csv_file = open(file_path, 'w')

        self.csv_writer = csv.writer(csv_file)
        self.csv_writer.writerow(['Time', 'X', 'Y', 'W', 'Xd', 'Yd', 'Wd'])

        self.prev_pose = None
        self.prev_time = None
        self.start_time = None
        self.callback_time = None
        self.alpha = 1.0 # No filter applied id alpha = 1.0

        self.subscription = rospy.Subscriber(
            '/vrpn_client_node/OBS/pose',
            PoseStamped,
            self.odom_callback,
            queue_size=10
        )

        self.odom_publisher = rospy.Publisher(
            'obstacle_odom', 
            Odometry, 
            queue_size=10)

        self.subscription = rospy.Subscriber(
            'emergency_flag',
            Bool,
            self.emergency_button_callback,
            queue_size=10)
    
    
    def emergency_button_callback(self, msg):
        if msg.data:
            rospy.loginfo('Obstacle Odom stopping by Emergency')
            rospy.signal_shutdown('Emergency stop')


    def calculate_euler_diff(self, current_orientation, previous_orientation):
        # Convert the quaternion objects to lists
        current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        previous_quaternion = [previous_orientation.x, previous_orientation.y, previous_orientation.z, previous_orientation.w]

        # Calculate the Euler angle differences
        current_euler = euler_from_quaternion(current_quaternion)
        previous_euler = euler_from_quaternion(previous_quaternion)
        euler_diff = [
            self.normalize_angle(current_euler[0] - previous_euler[0]),
            self.normalize_angle(current_euler[1] - previous_euler[1]),
            self.normalize_angle(current_euler[2] - previous_euler[2])
        ]

        return euler_diff


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


    def loop(self, event):

        current_time = time.time()

        # If the time of the callback is essentially the same as the control loop time
        if self.callback_time is None:
            return

        if self.prev_pose is not None and self.prev_time is not None:
            if self.prev_pose == self.pose:
                return

            time_diff = current_time - self.prev_time

            # Calculate linear velocity
            xd = (self.x - self.prev_pose.position.x) / time_diff
            yd = (self.y - self.prev_pose.position.y) / time_diff

            # Low-pass filter the velocity data
            self.filtered_xd = self.alpha * xd + (1 - self.alpha) * self.filtered_xd
            self.filtered_yd = self.alpha * yd + (1 - self.alpha) * self.filtered_yd

            # Convert Euler angles (roll, pitch, yaw) to angular velocities
            euler_diff = self.calculate_euler_diff(self.orientation, self.prev_pose.orientation)

            # Calculate angular velocity
            wd = euler_diff[2] / time_diff

            # Low-pass filter the angular velocity data
            self.filtered_wd = self.alpha * wd + (1 - self.alpha) * self.filtered_wd

        else:
            self.filtered_xd = 0.0
            self.filtered_yd = 0.0
            self.filtered_wd = 0.0

        self.prev_pose = self.pose
        self.prev_time = current_time

        # Write the data to the CSV file
        self.csv_writer.writerow([current_time, self.x, self.y, self.w, self.filtered_xd, self.filtered_yd, self.filtered_wd])
        
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        
        # Populate the position and velocity information
        self.odom.pose.pose = Pose(Point(self.x, self.y, self.z), self.orientation)
        self.odom.twist.twist = Twist(Vector3(self.filtered_xd, self.filtered_yd, 0.0), Vector3(0.0, 0.0, self.filtered_wd))

        # Publish the odometry message
        self.odom_publisher.publish(self.odom)


    def odom_callback(self, msg):
        # Get the current timestamp
        current_time = rospy.Time.now()

        self.callback_time = time.time()

        if self.start_time is None:
            self.start_time = current_time

        # Calculate elapsed time
        self.elapsed_time = (current_time - self.start_time).to_sec()

        # Process the pose data
        pose = msg.pose
        self.pose = pose

        # Process the pose data
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z

        orientation = pose.orientation
        self.orientation = orientation
        _, _, self.w = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        # self.publisher.publish(msg)


def main():
    odom_publisher = OdomPublisher()
    rospy.spin()
    odom_publisher.csv_file.close()


if __name__ == '__main__':
    main()