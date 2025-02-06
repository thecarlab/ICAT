import numpy as np
from math import pi,cos,sin, atan2
from nav_gym.map.util import load_img
import matplotlib.pyplot as plt
from topo import *
from nav_gym.obj.geometry.util import topi
# Define the points


# ICAT real size 5.22 m * 5.78 m
ndt_pts = np.array([
    (5.13, 2.480),
    (3.55, 0.19),
    (2.40, -0.20),
    (0.0, -0.715),
    (0.52, 2.42),
    (-0.09, 4.33),
    (2.20, 3.82),
    (5.61, 4.42),
    (2.82,1.77)
])

icat_pts = np.array([
    (5.87, 19.2),
    (23.36, 45.16),
    (35.56, 45.23),
    (60, 50),
    (54.33, 19.55),
    (59.56, 0.42),
    (36.3, 5.1),
    (0.71, 0.49),
    (30, 25.4)
])

icat_pts[:,0] *= 5.78/6
icat_pts[:,1] *= 1.0151
icat_centered = icat_pts - icat_pts[-1]

print("icat_pts: ", icat_pts )

import numpy as np

def calc_inverse_transformed_points(icat_pts, scaling_factor, theta, dx, dy, rotation_center, offset):
    # Inverse Translation
    translated_points = icat_pts - np.array([dx, dy])
    rotation_center = rotation_center  - np.array([dx,dy])
    # Creating Inverse Rotation Matrix
    inv_rotation_matrix = np.array([
                                    [np.cos(-theta), -np.sin(-theta)],
                                    [np.sin(-theta), np.cos(-theta)]
                                  ])

    # Adjusting for the original last point translation
    rotated_points = translated_points - rotation_center
    
    # Inverse Rotation
    rotated_points = np.dot(rotated_points, inv_rotation_matrix)
    
    # Revert the adjustment for the original last point translation
    rotated_points = rotated_points + rotation_center

    # Inverse Scaling
    final_pts = rotated_points / scaling_factor

    final_pts += offset


    return final_pts
    # return translated_points
    # return rotated_points


def calc_tranformed_points(ndt_pts, scaling_factor, theta, dx,dy):
    scaled_ndt_pts =  ndt_pts *scaling_factor
    rotation_matrix = np.array([
                                    [cos(theta), -sin(theta)],
                                    [sin(theta), cos(theta)]
                                ])
    translated_points = scaled_ndt_pts - scaled_ndt_pts[-1]
    rotated_points = np.dot(translated_points, rotation_matrix)
    rotated_points = rotated_points + scaled_ndt_pts[-1]
    final_pts = rotated_points+ np.array([dx,dy])
    return final_pts


def get_transformed_nodes(node_list , transformation, offset):
    # scaling_factor, theta, dx, dy = best_param
    # scaling_factor, theta, dx, dy = (10.193, 3.18, 2.0, 8.0)
    scaling_factor, theta, dx, dy = transformation
    # scaling_factor, theta, dx, dy = (10.1, 3.17, 2.0, 8.0)
    # scaling_factor, theta, dx, dy = (10.0, 3.20, 0.0, 10.0)
    final_pts = calc_tranformed_points(ndt_pts, scaling_factor, theta, dx,dy)
    inverse_pts = calc_inverse_transformed_points(icat_pts, scaling_factor, theta, dx,dy, icat_pts[-1], offset)

    node_list = node_list.copy()
    edge_list = get_edge_list(node_list=node_list)

    node_pts = []
    for node in node_list:
        node_pts.append(np.array(node[1]["coord"][:2]))
    # node_pts.append(np.array([30, 25.4]))
    node_pts = np.array(node_pts)
    print("node_pts: ", node_pts)
    transformed_pts = calc_inverse_transformed_points(node_pts,scaling_factor, theta, dx,dy, rotation_center = icat_pts[-1], offset=offset)
    dx, dy = transformed_pts[1] - transformed_pts[0]
    theta = atan2(dy,dx)
    print(" node 1 and 2 yaw angle: ", theta)
    yaw25_24 = -pi/2 + theta
    print(" yaw25_24 {}, topi {} ".format(yaw25_24, yaw25_24+2*pi))


    for i in range(len(node_list)):
        _, _, yaw = node_list[i][1]["coord"]
        x,y = transformed_pts[i]
        node_list[i][1]["coord"] = (x,y, topi(yaw+ theta))

    print("node list: ", node_list)
    return node_list






# scaling_factor, theta, dx, dy = (10.193, 3.18, 2.0, 8.0)
T = (10.00, 3.140, 2.0, 8.0)
icat_offset = np.array([0.18,-0.15])
icat_offset = np.array([0.18,-0.15]) + np.array([0. , + 0.2])
scaling_factor, theta, dx, dy = T

node_list = get_tuned_node_list()


transformed_nodes = get_transformed_nodes(node_list=node_list, transformation=T, offset= icat_offset)
# print("transformed_nodes: ", transformed_nodes)

icat_edge_list = get_edge_list(transformed_nodes, interval= 0.05)
# print("icat edge list: ", icat_edge_list)

# **************** Save Edges and Nodes ***************************
save_edges('/home/tian/icat_edges.json', icat_edge_list)

save_edges('/home/tian/icat_nodes.json', transformed_nodes)

transformed_pts, _ = get_points_from_nodes(transformed_nodes)
transformed_edge_pts, angles = get_points_from_edges(icat_edge_list)


fig,ax = plt.subplots()

plt.scatter(transformed_pts[:, 0], transformed_pts[:, 1], color='pink', label='Rotated NDT Points')


# plt.scatter(transformed_edge_pts[:, 0], transformed_edge_pts[:, 1], color='red', label='Rotated edge Points')

recorded_points = np.load('/home/tian/track_points.npy')
recorded_points = recorded_points[:int(len(recorded_points)/2)]
# print(" ******** recorded points *******************")
# print(len(recorded_points))
# print()
plt.scatter(recorded_points[:, 0], recorded_points[:, 1], color='red', label='Recoreded Points')

for i in range(len(transformed_edge_pts)):
    plt.arrow(transformed_edge_pts[:, 0][i], transformed_edge_pts[:, 1][i],0.05*cos(angles[i]), 0.05*sin(angles[i]))
# img = load_img('icat.png')
# plt.imshow(img, origin='lower', extent=[0, 60, 0, 50])  # The extent parameter sets the axis limits and 'origin' is now set to 'lower'.
# plt.imshow(img, extent=[0, 60, 0, 50])
# Labels and legend


x1 = [0.520, 0.510]; y1 = [3.121, 0.155]
x2 = [5.055, 5.164]; y2 = [0.190,3.267]
x3 = [1.082, 4.744]; y3 = [3.826,3.814]
x4 = [0.949, 4.642]; y4 = [-0.230, -0.236]
key_pts = np.array([[0.520,3.121], [0.510, 0.155],
                    [5.055, 0.190], [5.164, 3.267],
                    [1.082, 3.826], [4.744, 3.814],
                    [0.949,-0.230], [4.642, -0.236]
                    ])
mid_pts = []
for i in range(len(key_pts)):
    if i%2 == 0:
        mid_pt = (key_pts[i]+key_pts[i+1])*0.5
        print("mid point : ", mid_pt)
        mid_pts.append(mid_pt)
for mid_pt in mid_pts:
    for m in mid_pts:
        plt.plot([mid_pt[0], m[0]], [mid_pt[1], m[1]])

for i in range(4):
    print(key_pts[2*i:2*i+1, 0])
    print(key_pts[2*i:2*i+1, 1])
    plt.plot(key_pts[2*i:2*i+1, 0], key_pts[2*i:2*i+1, 1])
plt.plot(x1, y1)
plt.plot(x2, y2)
plt.plot(x3, y3)
plt.plot(x4, y4)
plt.plot([1.082, 0.949], [3.826, -0.230])
plt.plot([4.744, 4.642], [3.814, -0.236])



plt.xlabel('X axis')
plt.ylabel('Y axis')
plt.title('Point Transformation Validation')
plt.legend()
plt.axis('equal')  # Equal scaling on both axes for correct aspect ratio


for node_id, data in enumerate(transformed_pts[:-1]):
    x,y = data
    ax.text(x, y, str(node_id), ha='center', va='center', color='orange')

# Show the plot
plt.show()