#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf.transformations as tf
import numpy as np
import math
import time

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')

        # Set the obstacle position
        self.obstacle_pos = Point(0.0, 0.0, 0.0)  # Example obstacle position

        # Create a publisher to send potential x and y colisions
        self.potential_pub = rospy.Publisher('/potential', Twist, queue_size=10)

        # Subscribe to laser scan data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Subscribe to robot pose date
        self.subscription = rospy.Subscriber(
            '/vrpn_client_node/P1/pose',
            PoseStamped,
            self.odom_callback,
            queue_size=10
        )
        
        self.timer = rospy.Timer(rospy.Duration(1/60), self.loop)
        
        self.odom = None
        
        self.prev_pose = None
        self.prev_time = None
        self.start_time = None
        self.callback_time = None
        
        ## constants
        self.n = 4.0
        self.a = 0.5
        self.b = 0.5
        self.k = 1.3
        
        rospy.loginfo("Obstacle avoidance node has been started!")

    def laser_callback(self, scan):
        # Calculate attractive force towards goal
        # self.goal_pos = Point(0.0, 0.0)  # Example goal position
        pass
        
    def odom_callback(self, odom):
        
        # Get the current timestamp
        current_time = rospy.Time.now()

        self.callback_time = time.time()

        if self.start_time is None:
            self.start_time = current_time

        # Calculate elapsed time
        self.elapsed_time = (current_time - self.start_time).to_sec()

        # Process the pose data
        odom = odom.pose
        self.odom = odom

        # Process the pose data
        self.position_x = odom.position.x
        self.position_y = odom.position.y
    
    def calculate_V(self, obstacle):
        return np.exp(-np.power((self.position_x - obstacle[0])/self.a, self.n))*np.exp(-np.power((self.position_y - obstacle[1])/self.b, self.n))
    
    def j_ob(self, v, obstacle):
        return -v * self.n * np.array([(np.power((self.position_x - obstacle[0]), self.n-1) / np.power(self.a, self.n)), np.power((self.position_y - obstacle[1]), self.n-1) / np.power(self.b, self.n)])


    def loop(self, event):
        
        if self.odom is None:
            #rospy.loginfo('oi')
            return
            
        twist_msg = Twist()
        
        obstacle = np.array([-1, 0.0])
        v = self.calculate_V(obstacle)
        #if np.linalg.norm([np.array([self.position_x, self.position_y]), obstacle]) < self.security_distance:
        if v > 0.01 :
            J_ob = self.j_ob(v, obstacle)
            v_ref = self.k * (-v)
            x_dot, y_dot = np.dot(np.linalg.pinv(J_ob.reshape(-1, 1)), v_ref)[0]
        else:
            x_dot, y_dot = 0.0, 0.0
        
        rospy.loginfo('x dot: ' + str(x_dot) + ', y dot: '+ str(y_dot))
        
        twist_msg.linear.x = x_dot  # Linear velocity in the x direction
        twist_msg.linear.y = y_dot  # Linear velocity in the y direction
        
        self.potential_pub.publish(twist_msg)


if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
