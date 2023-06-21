#!/usr/bin/env python3

import rospy
import tf.transformations as tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from math import cos, sin, pi
import numpy as np
import time
import math


class PioneerController:
    def __init__(self):
        rospy.init_node('pioneer_controller')

        self.publisher = rospy.Publisher(
            '/RosAria/cmd_vel',
            Twist,
            queue_size=10)

        self.pose_subscriber = rospy.Subscriber(
            "/vrpn_client_node/P1/pose",
            PoseStamped,
            self.RobotPose,
            queue_size=40)

        self.subscription = rospy.Subscriber(
            'path_pose',
            Odometry,
            self.Path,
            queue_size=10)

        self.subscription = rospy.Subscriber(
            'emergency_flag',
            Bool,
            self.emergency_button_callback,
            queue_size=10)
        
        self.subscription = rospy.Subscriber(
            'potential',
            Twist,
            self.obstacle_avoidance_callback,
            queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1/30), self.control_loop)

        self.pgains = [1.5, 1, 1.5, 1]
        self.a = 0.3

        self.prev_pose = None
        self.prev_time = None

        # Initializing emergency button to False
        self.btn_emergencia = False
        
        # Initializing path variables
        self.path_x = 0.0
        self.path_y = 0.0
        self.path_z = 0.0

        self.path_linear_x = 0.0
        self.path_linear_y = 0.0
        self.path_linear_z = 0.0

        self.path_angular_x = 0.0
        self.path_angular_y = 0.0
        self.path_angular_z = 0.0
        
        #### OBSTACULO
        self.obstacle_avoidance = None

    def RobotPose(self, msg):
        # Process the pose data
        pose = msg.pose

        # Process the pose data
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y
        self.robot_z = pose.position.z

        orientation = pose.orientation
        self.robot_roll, self.robot_pitch, self.robot_yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
         
        

        if self.prev_pose is not None and self.prev_time is not None:
            current_time = time.time()
            time_diff = current_time - self.prev_time

            # Calculate linear velocity
            self.robot_linear_x = (msg.pose.position.x - self.prev_pose.position.x) / time_diff
            self.robot_linear_y = (msg.pose.position.y - self.prev_pose.position.y) / time_diff
            self.robot_linear_z = (msg.pose.position.z - self.prev_pose.position.z) / time_diff

            # Convert Euler angles (roll, pitch, yaw) to angular velocities
            euler_diff = self.calculate_euler_diff(msg.pose.orientation, self.prev_pose.orientation)

            # Calculate angular velocity
            self.robot_roll_velocity = euler_diff[0] / time_diff
            self.robot_pitch_velocity = euler_diff[1] / time_diff
            self.robot_yaw_velocity = euler_diff[2] / time_diff

        self.prev_pose = msg.pose
        self.prev_time = time.time()

    def calculate_euler_diff(self, current_orientation, previous_orientation):
        # Convert the quaternion objects to lists
        current_quaternion = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]
        previous_quaternion = [previous_orientation.x, previous_orientation.y, previous_orientation.z, previous_orientation.w]

        # Calculate the Euler angle differences
        current_euler = tf.euler_from_quaternion(current_quaternion)
        previous_euler = tf.euler_from_quaternion(previous_quaternion)
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

    def Path(self, msg):
        pose = msg.pose.pose
        velocity = msg.twist.twist

        # Process the pose and velocity data
        position = pose.position
        orientation = pose.orientation

        linear_velocity = velocity.linear
        angular_velocity = velocity.angular

        # Access specific components of the pose and velocity
        self.path_x = position.x
        self.path_y = position.y
        self.path_z = position.z

        # Convert the transformed orientation quaternion to Euler angles
        roll, pitch, yaw = tf.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self.path_linear_x = linear_velocity.x
        self.path_linear_y = linear_velocity.y
        self.path_linear_z = linear_velocity.z

        self.path_angular_x = angular_velocity.x
        self.path_angular_y = angular_velocity.y
        self.path_angular_z = angular_velocity.z

    def emergency_button_callback(self, msg):
        if msg.data:
            self.btn_emergencia = True

    def control_loop(self, event):
        if self.prev_pose is None:
            return

        desired_linear_velocity, desired_angular_velocity = self.controller()

        #rospy.loginfo('Linear Velocity: ' + str(desired_linear_velocity) + ', Angular Velocity: ' + str(desired_angular_velocity))

        ctrl_msg = Twist()
        ctrl_msg.linear.x = desired_linear_velocity
        ctrl_msg.linear.y = 0.0
        ctrl_msg.linear.z = 0.0
        ctrl_msg.angular.x = 0.0
        ctrl_msg.angular.y = 0.0
        ctrl_msg.angular.z = desired_angular_velocity
        # Publish the Twist message to control the robot
        self.publisher.publish(ctrl_msg)

        if self.btn_emergencia:
            rospy.loginfo('Robot stopping by Emergency')
            rospy.loginfo('Sending emergency stop command')

            for _ in range(10):
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.linear.z = 0.0
                stop_cmd.angular.x = 0.0
                stop_cmd.angular.y = 0.0
                stop_cmd.angular.z = 0.0
                # Publish the Twist message to stop the robot
                self.publisher.publish(stop_cmd)

            rospy.signal_shutdown("Emergency stop")
    
    def obstacle_avoidance_callback(self, ob_avoidance):
        self.obstacle_avoidance = ob_avoidance
        
        self.x_dot_avoidance = ob_avoidance.linear.x
        self.y_dot_avoidance = ob_avoidance.linear.y

    def controller(self):
        Kp = np.array([[self.pgains[0], 0],
                       [0, self.pgains[1]]])

        K = np.array([[np.cos(self.robot_yaw), -self.a*np.sin(self.robot_yaw)],
                      [np.sin(self.robot_yaw), self.a*np.cos(self.robot_yaw)]])
                      
        #K = np.array([[1, 0], 
        #             [0, 1]])

        Xtil = np.array([self.path_x - self.robot_x, self.path_y - self.robot_y])
        
        if self.obstacle_avoidance is not None:
            desired_velocity = np.array([self.path_linear_x + self.x_dot_avoidance, self.path_linear_y + self.y_dot_avoidance])
            rospy.loginfo(str(self.x_dot_avoidance))
        else:
            desired_velocity = np.array([self.path_linear_x, self.path_linear_y])

        reference_velocity = np.dot(np.linalg.inv(K), (desired_velocity.T + np.dot(Kp, Xtil.T)))
        
        #rospy.loginfo(str(self.robot_x - self.path_x) + ' ' + str(Xtil[0]) + ' ' +  str(Xtil[1]))

        desired_linear_velocity = reference_velocity[0]
        desired_angular_velocity = reference_velocity[1]

        if np.abs(desired_linear_velocity) > 0.5:
            desired_linear_velocity = np.sign(desired_linear_velocity)*0.5

        if np.abs(desired_angular_velocity) > 0.5:
            desired_angular_velocity = np.sign(desired_angular_velocity)*0.5

        return desired_linear_velocity, desired_angular_velocity


def main():
    rospy.init_node('pioneer_controller')

    controller = PioneerController()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()