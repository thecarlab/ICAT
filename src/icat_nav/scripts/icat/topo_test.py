#!/usr/bin/env python3
# encoding: utf-8

from topo import *
# from nav_gym.map.util import load_img
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Arrow
from quintic import QuinticPolynomial, quintic_planner, quintic_plan
import os
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('icat_nav')

def load_img(name = 'icat.png'):
    img_path = package_path + '/map/' + name
    img = plt.imread(img_path)
    return img


def view_topo(node_lsit, if_arrow = False):
    
    edge_list = get_edge_list(node_list=node_list)
    for edge in edge_list:
        print("----------------------")
        print("----> ", edge)

    fig,ax = plt.subplots()
    img = load_img('icat.png')

    for node_id, data in node_list:
        x, y, yaw = data["coord"]
        
        # Differentiate color based on "itsc" property
        color = 'green' if data["itsc"] else 'blue'
        
        # Draw the disk patch (circle)
        circle = Circle((x, y), radius=1, color=color, ec="black")  # ec stands for edgecolor
        ax.add_patch(circle)
        
        # Add node ID inside the circle
        ax.text(x, y, str(node_id), ha='center', va='center', color='white')

    for edge in edge_list:
        points = edge[2]["waypoints"]
        #points = [(10, 10), (30, 40), (55, 5), (5, 45)]

        # Unzip the points to separate x and y coordinates for easy plotting
        x_coords, y_coords, yaw = zip(*points)

        # Plotting the points
        plt.scatter(x_coords, y_coords, color='red')  # You can change the color and other properties as needed.
        # Drawing lines connecting the points
        plt.plot(x_coords, y_coords, color='red')  # You can change the color and other properties as needed.

    if if_arrow:
        for edge in edge_list:
            points = edge[2]["waypoints"]
            for x, y, yaw in points:
                arrow_length = 0.5  # Adjust as needed
                # plt.scatter(x, y, color='red')
                dx = arrow_length * np.cos(yaw)
                dy = arrow_length * np.sin(yaw)
                
                arrow = Arrow(x, y, dx, dy, width=0.2, color='red')  # Adjust width and color as needed
                ax.add_patch(arrow)

    # Plotting the image
    # plt.imshow(img, origin='lower', extent=[0, 60, 0, 50])  # The extent parameter sets the axis limits and 'origin' is now set to 'lower'.
    plt.imshow(img, extent=[0, 60, 0, 50]) 
    # Setting the x and y limits for the axes
    plt.xlim(0, 60)
    plt.ylim(0, 50)

    # Displaying the plot
    plt.show()

def view_trajectory(if_arrow = True):
    node_list = get_node_list()
    edge_list = get_edge_list(node_list=node_list)
    G = build_graph(node_list, edge_list)
    for edge in edge_list:
        print("----------------------")
        print("----> ", edge)

    fig,ax = plt.subplots()
    img = load_img('icat.png')

    for node_id, data in node_list:
        x, y, yaw = data["coord"]
        # Differentiate color based on "itsc" property
        color = 'green' if data["itsc"] else 'blue'
        # Draw the disk patch (circle)
        circle = Circle((x, y), radius=1, color=color, ec="black")  # ec stands for edgecolor
        ax.add_patch(circle)
        # Add node ID inside the circle
        ax.text(x, y, str(node_id), ha='center', va='center', color='white')
    for edge in edge_list:
        points = edge[2]["waypoints"]
        x_coords, y_coords, yaw = zip(*points)
    if if_arrow:
        for edge in edge_list:
            points = edge[2]["waypoints"]
            for x, y, yaw in points:
                arrow_length = 0.5  # Adjust as needed
                # plt.scatter(x, y, color='red')
                dx = arrow_length * np.cos(yaw)
                dy = arrow_length * np.sin(yaw)
                arrow = Arrow(x, y, dx, dy, width=0.2, color='red')  # Adjust width and color as needed
                ax.add_patch(arrow)


    xs, ys, yaws = G.get_edge_data(2,19)["waypoints"][10]
    xe, ye, yawe = G.get_edge_data(2,19)["waypoints"][-1]
    vs = 5.0; ve = 0.0
    a_s = 0.0; ae = 0.0
    a_max = 3.0; jerk_max = 5.0
    T = 6.0

    print(xs, ys, yaws)
    print(xe, ye, yawe )
    # quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    time, rx, ry, ryaw, rv, ra, rj = quintic_plan(xs,ys,yaws,vs,a_s,xe,ye,yawe, ve,ae, a_max, jerk_max, dt = 0.1, T= T)
    print("trjectory time is: ", time)
    print("speed profiling: ",rv)
    # **** plot trajectory ***
    arrow_dx = np.cos(ryaw)
    arrow_dy = np.sin(ryaw)
    # Plot trajectory points
    plt.scatter(rx, ry, color='blue', s=10, label='Trajectory points')
    # Plot arrows indicating yaw angles
    # plt.quiver(rx, ry, arrow_dx, arrow_dy, angles='xy', scale_units='xy', scale=1, color='red', width=0.005)
    # **** ****

    plt.imshow(img, extent=[0, 60, 0, 50]) 
    # Setting the x and y limits for the axes
    plt.xlim(0, 60)
    plt.ylim(0, 50)
    # Displaying the plot
    plt.show()




def test_save_edges():
    filename = "edges.json"
    node_list = get_node_list()
    edge_list = get_edge_list(node_list=node_list)
    save_edges(filename, edge_list)

def test_graph():
    G = build_graph()
    print(G)
    path, cost= A_star_path(G, 1, 2)
    print("cost: ", cost, path)

def test_load_edges():
    filename = "edges.json"
    edges = load_edges(filename)
    print("-----------------Testing Loading Edge:")
    print("len of edges: ", len(edges))
    print("edge[0]:  ", edges[0])

def localize_to_road():
    in_edge = (-1, -1)
    # path = self.PathBuffer[car_id]
    # path = [12, 31, 32, 33]
    # path = [23, 30, 29, 19, 20, 33]
    # path = [12, 31, 32, 33, 34, 24, 23, 30, 29]
    path = [42, 26, 25, 24, 23, 30, 29, 19, 20]

    print("path: ", path)
    # state = (49.508466103458105, 6.3847133021489935)
    # state = (12.895268896373137, 24.36347756274566)
    state = (53.18092806342177, 40.09633967926529)
    # state = (12.902820048070506, 24.331079388536846)
    # print("path: ", path)
    # print("car state: ",state)
    node_list = get_node_list()
    edge_list = get_edge_list(node_list)
    G = build_graph(node_list, edge_list)
    for n in range(len(path)-1):
        edge_points = G.get_edge_data(path[n],path[n+1])["waypoints"]
        # print("section points: ", edge_points)
        closest_index = find_closest_waypoint(state[0], state[1], edge_points)
        closest_point = edge_points[closest_index]
        s,d = frenet_transform(state[0], state[1], edge_points)
        if n == 0 and s < 0 :
            in_edge = (-1, path[n])
            print(" In edge: ", path[n],path[n+1])
            return s,d, in_edge, closest_index, closest_point
        if s >= 0 and s < G.get_edge_data(path[n],path[n+1])["weight"]:
            in_edge = (path[n], path[n+1])
            print(" In edge: ", path[n],path[n+1])
            return s,d,in_edge, closest_index, closest_point
        
        print("the closest index: ", closest_index)
        print(" print s,d : ", s,d)
    if in_edge == (-1,-1):
        # print("car id , ", id)
        print("car's path: ", path)
        # print("car state: ", self.StateBuffer[car_id])
        raise ValueError (" Cannot localize to the path !")

# view_trajectory()    
# test_graph()
# node_list = get_node_list()
# node_list = get_tuned_node_list()
node_list = get_node_list()
view_topo(node_list , if_arrow=False)
# test_save_edges()
# test_load_edges()
# localize_to_road()



