import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Set the backend to TkAgg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Initialize figure and axis
fig, ax = plt.subplots()

# Initialize an empty line object
line, = ax.plot([], [])

# Set the x and y limits of the plot
ax.set_xlim(0, 10)
ax.set_ylim(-1, 1)

# Define the initialization function
def init():
    line.set_data([], [])
    return line,

# Define the update function
def update(frame):
    x = np.linspace(0, 10, 100000)
    #print(frame)
    y = np.sin(2 * np.pi * (x - 0.1 * frame))
    line.set_data(x, y)
    return line,

# Create the animation
ani = FuncAnimation(fig, update, frames=10000, init_func=init, blit=True)

# Show the plot
plt.show()