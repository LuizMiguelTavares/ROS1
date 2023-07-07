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

class DroneController:
    def __init__(self):
        # Initialize the Bebop drone controller with default values
        self.bebop = Bebop([0]*18, [0]*18, [0.5, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5])
        
        # Subscribers for the drone's position and the desired path
        self.pose_subscriber = rospy.Subscriber("vrpn_client_node/B1/pose", PoseStamped, self.pose_callback)
        self.path_pose_subscriber = rospy.Subscriber("B1/path_pose", PoseStamped, self.path_pose_callback)
        self.path_velocity_subscriber = rospy.Subscriber("B1/path_velocity", TwistStamped, self.path_velocity_callback)
        self.path_acceleration_subscriber = rospy.Subscriber("B1/path_acceleration", AccelStamped, self.path_acceleration_callback)
        # Subscriber for the emergency flag
        self.emergency_subscriber = rospy.Subscriber("emergency_flag", Bool, self.emergency_callback)

        # Publishers for the takeoff, landing, and movement commands
        self.takeoff_publisher = rospy.Publisher("B1/takeoff", Empty, queue_size=10)
        self.land_publisher = rospy.Publisher("B1/land", Empty, queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher("B1/cmd_vel", Twist, queue_size=10)

        # Initial placeholders for pose and path data
        self.pose = PoseStamped()
        self.path_pose = PoseStamped()
        self.path_velocity = TwistStamped()
        self.path_acceleration = AccelStamped()

        # Control loop rate (30 Hz)
        self.rate = rospy.Rate(30) 

    # Callback functions to handle received messages
    def pose_callback(self, data):
        self.pose = data

    def path_pose_callback(self, data):
        self.path_pose = data

    def path_velocity_callback(self, data):
        self.path_velocity = data

    def path_acceleration_callback(self, data):
        self.path_acceleration = data

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

            # Update Bebop state
            self.bebop.set_pose(self.pose.pose.position.x,
                                self.pose.pose.position.y,
                                self.pose.pose.position.z,
                                self.get_euler_from_quaternion(self.pose.pose.orientation)[0],
                                self.get_euler_from_quaternion(self.pose.pose.orientation)[1],
                                self.get_euler_from_quaternion(self.pose.pose.orientation)[2],
                                vel[0], vel[1], vel[2],
                                acc[0], acc[1], acc[2],
                                ang_vel[0], ang_vel[1], ang_vel[2],
                                ang_acc[0], ang_acc[1], ang_acc[2])

            # Update Bebop desired state
            self.bebop.set_desired(self.path_pose.pose.position.x,
                                   self.path_pose.pose.position.y,
                                   self.path_pose.pose.position.z,
                                   self.get_euler_from_quaternion(self.path_pose.pose.orientation)[0],
                                   self.get_euler_from_quaternion(self.path_pose.pose.orientation)[1],
                                   self.get_euler_from_quaternion(self.path_pose.pose.orientation)[2],
                                   self.path_velocity.twist.linear.x,
                                   self.path_velocity.twist.linear.y,
                                   self.path_velocity.twist.linear.z,
                                   self.path_velocity.twist.angular.x,
                                   self.path_velocity.twist.angular.y,
                                   self.path_velocity.twist.angular.z,
                                   self.path_acceleration.accel.linear.x,
                                   self.path_acceleration.accel.linear.y,
                                   self.path_acceleration.accel.linear.z,
                                   self.path_acceleration.accel.angular.x,
                                   self.path_acceleration.accel.angular.y,
                                   self.path_acceleration.accel.angular.z)

            # Compute control commands
            u_theta, u_phi, u_z_dot, u_psi_dot = self.bebop.compute_controller()

            # Construct Twist message and publish to command velocity topic
            twist_cmd = self.construct_twist_msg(u_theta, u_phi, u_z_dot, u_psi_dot)
            self.cmd_vel_publisher.publish(twist_cmd)

            # Sleep to maintain the loop rate
            self.rate.sleep()

    # Function to create a Twist message from the control commands
    def construct_twist_msg(self, u_theta, u_phi, u_z_dot, u_psi_dot):
        twist = Twist()
        twist.linear.x = u_theta
        twist.linear.y = u_phi
        twist.linear.z = u_z_dot
        twist.angular.z = u_psi_dot
        return twist

    # Function to convert quaternion to Euler angles
    def get_euler_from_quaternion(self, quaternion):
        euler = tf_trans.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler

# Main function to create a node and start the control loop
if __name__ == "__main__":
    rospy.init_node('bebop_controller')
    controller = DroneController()
    controller.start()