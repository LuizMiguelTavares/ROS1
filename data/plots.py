import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Example with additional parameters
circular_path = pd.read_csv('data/circular_path_data.csv', delimiter=',', encoding='utf-8', header=0)
pioneer_odom = pd.read_csv('data/pioneer_odom.csv', delimiter=',', encoding='utf-8', header=0)
obstacle_odom = pd.read_csv('data/obstacle_odom.csv', delimiter=',', encoding='utf-8', header=0)
obstacle_detection_pioneer = pd.read_csv('data/obstacle_detection_pioneer.csv', delimiter=',', encoding='utf-8', header=0)

def find_closest_arg(arr, target):
    # Expand dimensions of the array and target for broadcasting
    arr = np.expand_dims(arr, axis=0)
    target = np.expand_dims(target, axis=1)
    
    # Calculate the absolute difference between the array and the target values
    diff = np.abs(arr - target)
    
    # Find the index of the minimum difference for each target value
    closest_arg = np.argmin(diff, axis=1)
    
    return closest_arg


def poda(*args, column='Time'):
    
    biggest_first_value = -np.inf
    smallest_last_value = np.inf

    smallest_dataset = np.inf

    for arg in args:
        length = len(arg)

        if length < smallest_dataset:
            smallest_dataset = length

        if arg[column].iloc[0] > biggest_first_value:
            biggest_first_value = arg[column].iloc[0]
        
        if arg[column].iloc[-1] < smallest_last_value:
            smallest_last_value = arg[column].iloc[-1]
    
    times = np.linspace(biggest_first_value, smallest_last_value, num=smallest_dataset)
    new_datasets = []

    fps = int(1/((smallest_last_value - biggest_first_value)/smallest_dataset))

    for arg in args:
        indices = find_closest_arg(arg[column], times)

        data = arg.iloc[indices]
        data.reset_index(drop=True, inplace=True)
        new_datasets.append(data)
    
    new_datasets.append(fps)
    return new_datasets

circular_path, pioneer_odom, obstacle_odom, obstacle_detection_pioneer, fps = poda(circular_path, pioneer_odom, obstacle_odom, obstacle_detection_pioneer)

# Filtering pionerr's Velocity
alpha = 0.2
for i in range(len(pioneer_odom)):
    # Low-pass filter the velocity data
    if i > 0:
        xd = pioneer_odom.iloc[i]['Xd']
        yd = pioneer_odom.iloc[i]['Yd']

        filtered_xd = pioneer_odom.iloc[i-1]['Xd']
        filtered_yd = pioneer_odom.iloc[i-1]['Yd']

        filtered_xd = alpha * xd + (1 - alpha) * filtered_xd
        filtered_yd = alpha * yd + (1 - alpha) * filtered_yd

        pioneer_odom.iloc[i]['Xd'] = filtered_xd
        pioneer_odom.iloc[i]['Yd'] = filtered_yd

Xtil = np.array([circular_path['X'] - pioneer_odom['X'], 
                 circular_path['Y'] - pioneer_odom['Y']]) 

vobs = np.array([obstacle_detection_pioneer['X_dot'], obstacle_detection_pioneer['Y_dot']]).T

nu_obs = (1 - np.abs(np.tanh(vobs)))

# Ganhos do controlador 
k1 = 0.3 # Ganho do Xtil
k2 = 1.5 # Ganho do obstáculo
k3 = 1   # GAnho interno do obstáculo

# Desired Velocity without the obstacle
Xtil = np.tanh(Xtil) * k1
path_velocity = np.array([circular_path['VX'], circular_path['VY']])
desired_velocity = (path_velocity + Xtil).T 

# Desired Velocity with the obstacle
Xtil_obs = np.tanh(Xtil) * nu_obs.T * k1
path_velocity_obs = np.array([circular_path['VX'], circular_path['VY']]) * nu_obs.T
obs_velocity = k2 * np.tanh(k3*vobs)

desired_velocity_obs = (path_velocity_obs + Xtil_obs + obs_velocity.T).T

def rotation_matrix(theta):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    rotation = np.array([[cos_theta, -sin_theta, 0],
                         [sin_theta, cos_theta, 0],
                         [0, 0, 1]])

    return rotation

pioneer_heigth = 0.7/2
pioneer_width = 0.6/2

obstacle_heigth = 0.3/2
obstacle_width = 0.3/2

thetas = np.linspace(0, 2 * np.pi, 100)
x_pioneer = pioneer_heigth * np.cos(thetas)
y_pioneer = pioneer_width * np.sin(thetas)

x_obstacle = obstacle_heigth * np.cos(thetas)
y_obstacle = obstacle_width * np.sin(thetas)

pioneer = np.array([x_pioneer, y_pioneer, np.ones(len(x_pioneer))])
obstacle = np.array([x_obstacle, y_obstacle, np.ones(len(x_obstacle))])

path_plot_x = []
path_plot_y = []

pioneer_plot_x = []
pioneer_plot_y = []

fig, ax = plt.subplots(figsize=(10, 10))

robots_and_obstacle_point_size = 4
obstacle_detection_point_size  = 7

def update_frame(i):
    #Clear Figure
    ax.clear()

    #Pioneer elipse
    rot_pioneer = np.dot(pioneer.T, rotation_matrix(-pioneer_odom.iloc[i]['W']))
    rot_pioneer_x = rot_pioneer[:, 0] + pioneer_odom.iloc[i]['X_controle']
    rot_pioneer_y = rot_pioneer[:, 1] + pioneer_odom.iloc[i]['Y_controle']
    ax.scatter(rot_pioneer_x, rot_pioneer_y, s=robots_and_obstacle_point_size)

    # Pioneer Control Path
    pioneer_plot_x.append(pioneer_odom.iloc[i]['X_controle'])
    pioneer_plot_y.append(pioneer_odom.iloc[i]['Y_controle'])
    ax.plot(pioneer_plot_x, pioneer_plot_y)

    velocity_module = np.linalg.norm([pioneer_odom.iloc[i]['Xd'], pioneer_odom.iloc[i]['Yd']])
    if velocity_module > 0.005:
        plt.arrow(pioneer_odom.iloc[i]['X_controle'], pioneer_odom.iloc[i]['Y_controle'], 
                pioneer_odom.iloc[i]['Xd'], 
                pioneer_odom.iloc[i]['Yd'], 
                length_includes_head=True, head_width=0.05, color='blue')

        # plt.arrow(pioneer_odom.iloc[i]['X_controle'], pioneer_odom.iloc[i]['Y_controle'], 
        #         desired_velocity_obs[i, 0], 
        #         desired_velocity_obs[i, 1], 
        #         length_includes_head=True, head_width=0.05, color='green') 
        
        if vobs[i, 0]>0 or vobs[i, 0]>0:
            plt.arrow(pioneer_odom.iloc[i]['X_controle'], pioneer_odom.iloc[i]['Y_controle'], 
                vobs[i, 0], 
                vobs[i, 1], 
                length_includes_head=True, head_width=0.05, color='red') 
            
            plt.arrow(pioneer_odom.iloc[i]['X_controle'], pioneer_odom.iloc[i]['Y_controle'], 
                desired_velocity[i, 0], 
                desired_velocity[i, 1], 
                length_includes_head=True, head_width=0.05, color='green') 

    # Path
    path_plot_x.append(circular_path.iloc[i]['X'])
    path_plot_y.append(circular_path.iloc[i]['Y'])
    ax.plot(path_plot_x, path_plot_y, 'g')

    # Obstacle
    rot_obstacle = np.dot(obstacle.T, rotation_matrix(-obstacle_odom.iloc[i]['W']))
    rot_obstacle_x = rot_obstacle[:, 0] + obstacle_odom.iloc[i]['X']
    rot_obstacle_y = rot_obstacle[:, 1] + obstacle_odom.iloc[i]['Y']
    ax.scatter(rot_obstacle_x, rot_obstacle_y, s=robots_and_obstacle_point_size)

    # Obstacle detection
    
    if np.abs(obstacle_detection_pioneer['X_dot'][i]) > 0.0 or np.abs(obstacle_detection_pioneer['Y_dot'][i]) > 0.0:
        ax.scatter(obstacle_detection_pioneer['X_robot_obs'][i], obstacle_detection_pioneer['Y_robot_obs'][i], s=obstacle_detection_point_size, c='r')
        ax.scatter(obstacle_detection_pioneer['X_obs_robot'][i], obstacle_detection_pioneer['Y_obs_robot'][i], s=obstacle_detection_point_size, c='r')

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])

    ax.set_aspect('equal')

    plt.draw()

# int(len(pioneer_odom)/2)

ani = FuncAnimation(fig, update_frame, frames=len(pioneer_odom), repeat=False)
ani.save('Elipse_melhor.gif', writer='pillow', fps=fps)