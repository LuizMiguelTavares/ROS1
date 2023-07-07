#!/usr/bin/env python3

# ROS Python client library
import rospy

# Imported classes and message types
from aurora_py.drones import Bebop
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped, Twist
from std_msgs.msg import Empty, Bool

# Python libraries for numerical operations
import numpy as np
import tf.transformations as tf_trans

import os
import csv

class DroneController:
    def __init__(self):
        # Initialize the Bebop drone controller with default values
        self.bebop = Bebop([0]*18, [0]*18, [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5])
        
        # Subscribers for the drone's position
        self.pose_subscriber = rospy.Subscriber("vrpn_client_node/B1/pose", PoseStamped, self.pose_callback)

        # Subscriber for the emergency flag
        self.emergency_subscriber = rospy.Subscriber("emergency_flag", Bool, self.emergency_callback)

        # Publishers for the takeoff, landing, and movement commands
        self.takeoff_publisher = rospy.Publisher("B1/takeoff", Empty, queue_size=10)
        self.land_publisher = rospy.Publisher("B1/land", Empty, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher("B1/cmd_vel", Twist, queue_size=10)

        # Initial placeholders for pose and path data
        self.pose = PoseStamped()

        # Create a CSV file to store the data
        folder_path = '/root/data/bebop/'

        # Create the file in the specified folder
        file_path = os.path.join(folder_path, 'bebop_odom.csv')
        self.csv_file = open(file_path, 'w')

        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(['Time', 'X', 'Y', 'Z','X_dot', 'Y_dot', 'Z_dot', 'X_ddot', 'Y_ddot', 'Z_ddot'])

        # Control loop rate (30 Hz)
        self.rate = rospy.Rate(30)

    # Callback functions to handle received messages
    def pose_callback(self, data):
        self.pose = data

    # Emergency callback to handle emergency landing situations
    def emergency_callback(self, data):
        if data.data:
            self.land_publisher.publish(Empty())
            rospy.signal_shutdown('Emergency landing')

    def start(self):
        pose_old = None
        time_old = None

        # Control loop
        while not rospy.is_shutdown():
            # Compute velocity and acceleration
            if pose_old is not None:
                dt = (self.pose.header.stamp - time_old).to_sec()
                vel = (np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z]) 
                        - np.array([pose_old.pose.position.x, pose_old.pose.position.y, pose_old.pose.position.z])) / dt
                ang_vel = (self.get_euler_from_quaternion(self.pose.pose.orientation) 
                            - self.get_euler_from_quaternion(pose_old.pose.orientation)) / dt
            else:
                vel = np.array([0.0, 0.0, 0.0])
                ang_vel = np.array([0.0, 0.0, 0.0])

            if pose_old is not None:
                acc = (vel - (np.array([pose_old.pose.position.x, pose_old.pose.position.y, pose_old.pose.position.z]) 
                        - np.array([pose_old.pose.position.x, pose_old.pose.position.y, pose_old.pose.position.z])) / dt) / dt
                ang_acc = (ang_vel - (self.get_euler_from_quaternion(self.pose.pose.orientation) 
                            - self.get_euler_from_quaternion(pose_old.pose.orientation)) / dt) / dt                                                                                                                                                                                                                                                                                                                                                                     
            else:
                acc = np.array([0.0, 0.0, 0.0])
                ang_acc = np.array([0.0, 0.0, 0.0])

            # Store current pose as old value for the next iteration
            pose_old = self.pose
            time_old = self.pose.header.stamp
            
            # Write the data to the CSV file
            self.csv_writer.writerow([time_old.to_sec(), 
                                      self.pose.pose.position.x,
                                      self.pose.pose.position.y,
                                      self.pose.pose.position.z, 
                                      vel[0], vel[1], vel[2],
                                      acc[0], acc[1], acc[2]])

            # Sleep to maintain the loop rate
            self.rate.sleep()

    # Function to convert quaternion to Euler angles
    def get_euler_from_quaternion(self, quaternion):
        euler = tf_trans.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler

# Main function to create a node and start the control loop
if __name__ == "__main__":
    rospy.init_node('bebop_controller')
    controller = DroneController()
    controller.start()
    controller.csv_file.close()