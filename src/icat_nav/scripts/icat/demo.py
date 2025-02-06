import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.interpolate import make_interp_spline
from nav_gym.map.util import load_map, load_img
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from nav_gym.obj.robot.robot import CarRobot, DiffRobot
from nav_gym.obj.robot.robot_param import CarParam
from nav_gym.sim.config import Config
from nav_gym.map.util import load_map
from nav_gym.obj.geometry.util import rot,line_line, line_polygon
from nav_gym.obj.geometry.objects import Polygon, build_wall
from nav_gym.sim.plot import plot_cars, plot_start_goal, plot_geo_disks, plot_VOs, plot_ORCA
from nav_gym.alg.ddpg.ddpg_torch import Agent2, Agent_VDN,Agent_VDN2,Agent_QMix, Agent_LagMix
from nav_gym.alg.rvo.rvo import RVO
from nav_gym.alg.rvo.orca import ORCA
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import random
import time
from sklearn.neighbors import KDTree
import gym
from gym import spaces
import os
import time
from nav_gym.obj.geometry.util import rot, topi
from nav_gym.sim.mix.buffer import Buffer
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline


current_file_path = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_path)
root_folder_path = os.path.dirname(current_folder_path)

fig,ax = plt.subplots()
img = load_img('icat.png')
# Display the image
plt.imshow(img, extent=[0, 60, 0, 50])
# plt.axis('off')  # Hide the axis values

# wall1 = np.array([[1.,5.],[6.,5.,],[6.,3.],[3., 4.]])
# walls = [wall1]
# wall_polygons = []
# for wall in walls:
#     wall_polygons.append(Polygon(wall)) 

# rect =  patches.Polygon(wall,linewidth=1, edgecolor='grey', facecolor='grey')
# ax.add_patch(rect)
# Define the points
# points = np.array([
#     [56.53, 11],
#     [56.53, 26],
#     [56.53, 41],
#     [50, 47],
#     [30, 47],
#     [10, 47],
#     [3.78, 41],
#     [3.78, 26],
#     [3.78, 11],
#     [10, 3.23],
#     [30, 3.23],
#     [48, 3.23],
#     [56.53, 11],  # Close the loop
# ])

# # Separate the points into x and y coordinates
# x, y = points[:, 0], points[:, 1]

# # Define the degree of the spline
# k = 3

# # Create the t values
# t = np.linspace(0, 1, len(x))

# # Create the B-spline
# tck = make_interp_spline(t, np.c_[x, y], k=k)
# tnew = np.linspace(0, 1, 100)
# xnew, ynew = tck(tnew).T

# # Compute the yaw angles
# tck_derivative = tck.derivative()
# dxnew, dynew = tck_derivative(tnew).T
# yaw_angles = np.arctan2(dynew, dxnew)

# # Define the car parameters
# car_length = 3
# car_width = 2
# num_cars = 3
# car_spacing = 10  # Number of interpolated points between each car

# # Initialize the plot
# # fig, ax = plt.subplots()

# # Plot the interpolated points and yaw angles
# plt.plot(xnew, ynew, '-', label='Path')

# # Create the car patches
# cars = [Rectangle((0, 0), car_length, car_width, fc='b') for _ in range(num_cars)]
# for car in cars:
#     ax.add_patch(car)

# plt.xlim(min(xnew) - 10, max(xnew) + 10)
# plt.ylim(min(ynew) - 10, max(ynew) + 10)
# plt.axis('equal')
# plt.legend()

# print(" ****************** len xnew: ****************** ", len(xnew))
# # Update the plot in each animation step
# for i in range(0, len(xnew), 1):
#     for j, car in enumerate(cars):
#         index = i - j * car_spacing
#         if index < 0:
#             index += len(xnew)
#         x, y, theta = xnew[index], ynew[index], yaw_angles[index]
#         car.set_xy((x - car_length / 2 * np.cos(theta), y - car_length / 2 * np.sin(theta)))
#         car.angle = np.degrees(theta)
#     plt.pause(1)
#     plt.draw()


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.interpolate import make_interp_spline

# Define the points
points = np.array([
    [56.53, 11],
    [56.53, 26],
    [56.53, 41],
    [50, 47],
    [30, 47],
    [10, 47],
    [3.78, 41],
    [3.78, 26],
    [3.78, 11],
    [10, 3.23],
    [30, 3.23],
    [48, 3.23],
    [56.53, 11],  # Close the loop
])

# Separate the points into x and y coordinates
x, y = points[:, 0], points[:, 1]

# Define the degree of the spline
k = 3

# Create the t values
t = np.linspace(0, 1, len(x))

# Create the B-spline
tck = make_interp_spline(t, np.c_[x, y], k=k)
tnew = np.linspace(0, 1, 100)
xnew, ynew = tck(tnew).T

# Compute the yaw angles
tck_derivative = tck.derivative()
dxnew, dynew = tck_derivative(tnew).T
yaw_angles = np.arctan2(dynew, dxnew)

# Define the car parameters
car_length = 3
car_width = 2
num_cars = 5
car_spacing = 5  # Number of interpolated points between each car

# Initialize the plot
# fig, ax = plt.subplots()

# Plot the interpolated points and yaw angles
plt.plot(xnew, ynew, '-', label='Path')

# Create the car patches
cars = [Rectangle((0, 0), car_length, car_width, fc='b') for _ in range(num_cars)]
for car in cars:
    ax.add_patch(car)

plt.xlim(min(xnew) - 10, max(xnew) + 10)
plt.ylim(min(ynew) - 10, max(ynew) + 10)
plt.axis('equal')
plt.legend()

# Update the plot in each animation step
for i in range(0, len(xnew), 1):
    for j, car in enumerate(cars):
        index = i - j * car_spacing
        if index < 0:
            index += len(xnew)
        x, y, theta = xnew[index], ynew[index], yaw_angles[index]
        # car.set_xy((x - car_length / 2 * np.cos(theta), y - car_length / 2 * np.sin(theta)))
        
        car.set_xy((x - car_length / 2 * np.cos(theta) + car_width / 2 * np.sin(theta), 
                    y - car_width / 2 * np.cos(theta) - car_length / 2 * np.sin(theta)))
        # car.set_xy((x, y))
        car.angle = np.degrees(theta)
    # plt.draw()
    plt.pause(0.01)

plt.show()