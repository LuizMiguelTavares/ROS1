#!/usr/bin/env python3
import numpy as np

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

    def min_distance(self, robot_pose, obstacle_pose):
        x_robot, y_robot, yaw_robot, height_robot, width_robot = robot_pose
        x_obstacle, y_obstacle, yaw_obstacle, height_obstacle, width_obstacle = obstacle_pose

        thetas = np.linspace(0, 2 * np.pi, 10)
        
        x = height_robot * np.cos(thetas)
        y = width_robot * np.sin(thetas)
        body = np.array([x, y, np.ones(len(x))])
        rot_body = np.dot(body.T, self.rotation_matrix(-yaw_robot))
        x_rot = rot_body[:, 0] + x_robot
        y_rot = rot_body[:, 1] + y_robot

        # Define the parameters of the ellipse obstacle
        x_ob = height_obstacle * np.cos(thetas)
        y_ob = width_obstacle * np.sin(thetas)
        body_obs = np.array([x_ob, y_ob, np.ones(len(x))])
        rot_obs = np.dot(body_obs.T, self.rotation_matrix(-yaw_obstacle))
        x_ob_rot = rot_obs[:, 0] + x_obstacle
        y_ob_rot = rot_obs[:, 1] + y_obstacle

        min_distance = np.inf

        for x, y in zip(x_ob_rot, y_ob_rot):
            distances = np.sqrt((x_rot - x)**2 + (y_rot - y)**2)
            if np.min(distances) < min_distance:
                min_distance = np.min(distances)
                x_ob_min = x
                y_ob_min = y
                closest_index = np.argmin(distances)
                closest_x, closest_y = x_rot[closest_index], y_rot[closest_index]
        
        x_diff = closest_x - x_ob_min
        y_diff = closest_y - y_ob_min
        return x_diff, y_diff

    def obstacle_avoidance(self, robot_pose, obstacle_pose):

        x_diff, y_diff = self.min_distance(robot_pose, obstacle_pose)

        v = self.calculate_V(x_diff, y_diff)

        if v > 0.01 :
            J_ob = self.j_ob(v, x_diff, y_diff)
            v_ref = self.k * (-v)
            x_dot, y_dot = np.dot(np.linalg.pinv(J_ob.reshape(-1, 1)), v_ref)[0]
        else:
            x_dot, y_dot = 0.0, 0.0
        
        #print('x dot: ' + str(x_dot) + ', y dot: '+ str(y_dot))
        return x_dot, y_dot

if __name__ == '__main__':
    pass