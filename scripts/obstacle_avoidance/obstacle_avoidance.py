#!/usr/bin/env python3
import numpy as np
from numba import jit

class ObstacleAvoidance:
    def __init__(self, n=None, a=None, b=None, k=None):

        ## constants
        self.n = n if n is not None else 4.0
        self.a = a if a is not None else 0.5
        self.b = b if b is not None else 0.5
        self.k = k if k is not None else 1.3

    def calculate_V(self, x_diff, y_diff):
        return np.exp(-np.power((x_diff)/self.a, self.n))*np.exp(-np.power((y_diff)/self.b, self.n))
    
    def j_ob(self, v, x_diff, y_diff):
        return -v * self.n * np.array([(np.power((x_diff), self.n-1) / np.power(self.a, self.n)), np.power((y_diff), self.n-1) / np.power(self.b, self.n)])
    
    def rotation_matrix(self, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        rotation = np.array([[cos_theta, -sin_theta, 0],
                            [sin_theta, cos_theta, 0],
                            [0, 0, 1]])

        return rotation
    
    
    def create_rounded_rectangle(self, width, height, radius, num_points=100):
        # Calculate the number of points per segment
        num_segment_points = num_points // 2

        # Calculate the angle increment for generating points on the corners
        angle_increment = np.pi / (2 * num_segment_points)

        # Generate points for the top side
        x_top = np.linspace(-width/2 + radius, width/2 - radius, num_segment_points)
        y_top = height/2 * np.ones_like(x_top)

        # Generate points for the bottom side
        x_bottom = np.linspace(width/2 - radius, -width/2 + radius, num_segment_points)
        y_bottom = -height/2 * np.ones_like(x_bottom)

        # Generate points for the left side
        y_left = np.linspace(-height/2 + radius, height/2 - radius, num_segment_points)
        x_left = -width/2 * np.ones_like(y_left)

        # Generate points for the right side
        y_right = np.linspace(height/2 - radius, -height/2 + radius, num_segment_points)
        x_right = width/2 * np.ones_like(y_right)

        # Generate points for the top-left corner
        theta = np.arange(np.pi, np.pi/2, -angle_increment)
        x_top_left = -width/2 + radius + radius * np.cos(theta)
        y_top_left = height/2 - radius + radius * np.sin(theta)

        # Generate points for the top-right corner
        theta = np.arange(np.pi/2, 0, -angle_increment)
        x_top_right = width/2 - radius + radius * np.cos(theta)
        y_top_right = height/2 - radius + radius * np.sin(theta)

        # Generate points for the bottom-left corner
        theta = np.arange(3*np.pi/2, np.pi, -angle_increment)
        x_bottom_left = -width/2 + radius + radius * np.cos(theta)
        y_bottom_left = -height/2 + radius + radius * np.sin(theta)

        # Generate points for the bottom-right corner
        theta = np.arange(2*np.pi, 3*np.pi/2, -angle_increment)
        x_bottom_right = width/2 - radius + radius * np.cos(theta)
        y_bottom_right = -height/2 + radius + radius * np.sin(theta)

        # Combine all the points to form the rounded rectangle
        x_points = np.concatenate([x_top, x_top_right, x_right, x_bottom_right,
                                x_bottom, x_bottom_left, x_left, x_top_left])
        y_points = np.concatenate([y_top, y_top_right, y_right, y_bottom_right,
                                y_bottom, y_bottom_left, y_left, y_top_left])

        return x_points, y_points


    def calculate_distance(self, point1, point2):
        """Calculate the Euclidean distance between two points."""
        x1, y1 = point1
        x2, y2 = point2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


    def find_closest_points(self, points1, points2):
        """Find the closest two points between two arrays of points."""
        closest_distance = 1e9  # A large constant representing infinity
        closest_points = None

        for p1 in points1:
            for p2 in points2:
                distance = self.calculate_distance(p1, p2)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_points = (p1, p2)

        return closest_points, closest_distance


    def min_distance(self, robot_pose, obstacle_pose):
        x_robot, y_robot, yaw_robot, height_robot, width_robot = robot_pose
        x_obstacle, y_obstacle, yaw_obstacle, height_obstacle, width_obstacle = obstacle_pose

        thetas = np.linspace(0, 2 * np.pi, 100)
        
        # x = height_robot * np.cos(thetas)
        # y = width_robot * np.sin(thetas)

        radius = 0.05
        x, y = self.create_rounded_rectangle(height_robot*2, width_robot*2, radius*2)
        
        body = np.array([x, y, np.ones(len(x))])
        rot_body = np.dot(body.T, self.rotation_matrix(-yaw_robot))
        x_rot = rot_body[:, 0] + x_robot
        y_rot = rot_body[:, 1] + y_robot

        robot_points = np.column_stack((x_rot, y_rot))

        # Define the parameters of the ellipse obstacle
        x_ob = height_obstacle * np.cos(thetas)
        y_ob = width_obstacle * np.sin(thetas)
        body_obs = np.array([x_ob, y_ob, np.ones(len(x_ob))])
        rot_obs = np.dot(body_obs.T, self.rotation_matrix(-yaw_obstacle))
        x_ob_rot = rot_obs[:, 0] + x_obstacle
        y_ob_rot = rot_obs[:, 1] + y_obstacle

        obstacle_points = np.column_stack((x_ob_rot, y_ob_rot))

        closest_points, distance = self.find_closest_points(robot_points, obstacle_points)

        closest_robot_points = closest_points[0]
        closest_x = closest_robot_points[0]
        closest_y = closest_robot_points[1]

        closest_obstacle_points = closest_points[1]
        x_ob_min = closest_obstacle_points[0]
        y_ob_min = closest_obstacle_points[1]
        
        pose_robot_min    = np.array([x_ob_min, y_ob_min])
        pose_obstacle_min = np.array([closest_x, closest_y])
        
        x_diff = closest_x - x_ob_min
        y_diff = closest_y - y_ob_min
        return x_diff, y_diff, pose_robot_min, pose_obstacle_min
        

    def obstacle_avoidance(self, robot_pose, obstacle_pose):

        x_diff, y_diff, pose_robot_min, pose_obstacle_min = self.min_distance(robot_pose, obstacle_pose)

        v = self.calculate_V(x_diff, y_diff)

        if v > 0.01 :
            J_ob = self.j_ob(v, x_diff, y_diff)
            v_ref = self.k * (-v)
            x_dot, y_dot = np.dot(np.linalg.pinv(J_ob.reshape(-1, 1)), v_ref)[0]
            
        else:
            x_dot, y_dot = 0.0, 0.0
            pose_robot_min, pose_obstacle_min = [None, None], [None, None]
        
        #print('x dot: ' + str(x_dot) + ', y dot: '+ str(y_dot))
        return x_dot, y_dot, pose_robot_min, pose_obstacle_min

if __name__ == '__main__':
    pass