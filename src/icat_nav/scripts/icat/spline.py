# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.interpolate import BSpline, make_interp_spline

# # Define the points
# points = np.array([
#     # right
#     [56.53, 11],
#     [56.53, 34], 
#     [56.53, 56],
#     # top
#     [50, 47],
#     [30, 47], 
#     [10, 47],
#     # left
#     [3.78, 56],
#     [3.78, 34],  
#     [3.78, 11],
#     # bot
#     [10, 3.23],
#     [30, 3.23], 
#     [50, 3.23],

# ])

# # Separate the points into x and y coordinates
# x, y = points[:, 0], points[:, 1]

# # Append the first point to the end to close the loop
# x = np.append(x, x[0])
# y = np.append(y, y[0])

# # Define the degree of the spline
# k = 3

# # Create the t values
# t = np.linspace(0, 1, len(x))

# # Create the B-spline
# tck = make_interp_spline(t, np.c_[x, y], k=k)
# tnew = np.linspace(0, 1, 100)
# xnew, ynew = tck(tnew).T

# # Plot the result
# plt.plot(x, y, 'o-', label='Original Points')
# plt.plot(xnew, ynew, '-', label='Interpolated Points')
# plt.legend()
# plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.interpolate import make_interp_spline
# import math

# # Define the points
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

# # Plot the result
# plt.plot(x, y, 'o-', label='Original Points')
# plt.plot(xnew, ynew, '-', label='Interpolated Points')
# plt.legend()
# plt.show()

# # Plot the yaw angles
# plt.plot(tnew, np.degrees(yaw_angles))
# plt.xlabel('t')
# plt.ylabel('Yaw Angle (degrees)')
# plt.title('Yaw Angles along the B-spline')
# plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.interpolate import make_interp_spline

# # Define the points
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

# # Combine the interpolated points and yaw angles into tuples
# interpolated_points_and_angles = [(x, y, theta) for x, y, theta in zip(xnew, ynew, yaw_angles)]

# # Plot the result
# plt.plot(x, y, 'o-', label='Original Points')
# plt.plot(xnew, ynew, '-', label='Interpolated Points')

# # Plot the arrows
# for x, y, theta in interpolated_points_and_angles[::10]:  # Skip some arrows for clarity
#     plt.arrow(x, y, np.cos(theta), np.sin(theta), head_width=1, head_length=1, fc='r', ec='r')

# plt.legend()
# plt.show()

import numpy as np
import matplotlib.pyplot as plt
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

    [56.53, 11],
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

# Combine the interpolated points and yaw angles into tuples
interpolated_points_and_angles = [(x, y, theta) for x, y, theta in zip(xnew, ynew, yaw_angles)]

# Plot the result
plt.plot(x, y, 'o-', label='Original Points')
plt.plot(xnew, ynew, '-', label='Interpolated Points')

# Plot the arrows
for x, y, theta in interpolated_points_and_angles:
    plt.arrow(x, y, np.cos(theta), np.sin(theta), head_width=1, head_length=1, fc='r', ec='r')

plt.legend()
plt.show()





