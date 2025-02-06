#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
import networkx as nx
import random
from topo import *
from car import build_car,get_car_param
from math import pi, sin, cos, atan2
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import bisect
from quintic import quintic_1d_plan
from plot import *
from traffic_manager import TrafficManager


WAYPOINT_DISTANCE = 0.05
N_CAR = 2
N_NODE = 42
CAR_PARAM = get_car_param()
CAR_INFO = {"hl":0.1775, "hw": 0.10, "amax":0.3, "amin":-0.3, "jerkmax": 1.0} # half length, half width of the car
DT = 0.1
N_LOOP = 10000

car_length = CAR_INFO["hl"]*2
car_width = CAR_INFO["hw"]*2
# node_list = get_tuned_node_list()
# edge_list = get_edge_list(node_list)
node_list = load_edges('/home/tian/icat_nodes.json')
edge_list = load_edges('/home/tian/icat_edges.json')
G = build_graph(node_list, edge_list)
node_arr = [i for i in range(1, N_NODE+1)]
# Sample nodes for start and goal
nodes = random.sample(node_arr, 2 * N_CAR)
start_nodes = nodes[:N_CAR]
goal_nodes = nodes[N_CAR:]
# start_nodes[0] = 5
# goal_nodes[0] = 6
start_nodes[0] = 16
goal_nodes[0] = 4
start_nodes[1] = 15
goal_nodes[1] = 20
# start_nodes = [21, 37, 28, 7, 39]
# goal_nodes = [19, 5, 4, 18, 36]
# Initalize Cars
cars = []
car_states = []
path_buffer = []
print(" *********************************")
# print("node list: ", node_list)
print("start nodes: ", start_nodes)
print("goal nodes: ", goal_nodes)
for i in range(N_CAR):
    n = start_nodes[i]
    coord = node_list[n-1][1]["coord"]
    print("car {}, start nodes id {}, coord {} ".format(i,n,coord))
    print(coord)
    # Add cars
    car = build_car(i, CAR_PARAM, coord)
    cars.append(car)
    car_states.append(car.state.copy())
    # Add paths
    path = nx.astar_path(G, start_nodes[i], goal_nodes[i], heuristic=None, weight="weight")
    path_buffer.append(path)
 
fig,ax = plt.subplots()



car_patches = [Rectangle((cars[i].state[0], cars[i].state[1]), CAR_INFO["hl"]*2, CAR_INFO["hw"]*2, fc='y') for i in range(N_CAR)]
for car_rect in car_patches:
    ax.add_patch(car_rect)
# plt.show()
for i in range(N_CAR):
    x, y, theta = car_states[i][:3]
    # car.set_xy((x - car_length / 2 * np.cos(theta), y - car_length / 2 * np.sin(theta)))
    
    car_patches[i].set_xy((x - car_length / 2 * np.cos(theta) + car_width / 2 * np.sin(theta), 
                y - car_width / 2 * np.cos(theta) - car_length / 2 * np.sin(theta)))
    # car.set_xy((x, y))
    car_patches[i].angle = np.degrees(theta)
# plt.show()


TM = TrafficManager(node_list=node_list, edge_list=edge_list, G = G, n_car = N_CAR,
                    car_states=car_states, car_info = CAR_INFO, start_nodes=start_nodes, goal_nodes=goal_nodes,
                    wpts_dist=WAYPOINT_DISTANCE)

sim_ctr = 0
T_history = []
for n_loop in range(N_LOOP):
    print(" *********************************** {} *********************************".format(n_loop))
    sim_ctr +=1 
   
    TM.traffic_state_update(car_states)
    # plot_cars(car_patches, car_states,CAR_INFO["hl"]*2, CAR_INFO["hw"]*2)
    for i in range(N_CAR):
        x, y, theta = car_states[i][:3]
        # car.set_xy((x - car_length / 2 * np.cos(theta), y - car_length / 2 * np.sin(theta)))
        
        car_patches[i].set_xy((x - car_length / 2 * np.cos(theta) + car_width / 2 * np.sin(theta), 
                    y - car_width / 2 * np.cos(theta) - car_length / 2 * np.sin(theta)))
        # car.set_xy((x, y))
        car_patches[i].angle = np.degrees(theta)

    plt.clf()
    # plt.xlim(0, 60)
    # plt.ylim(0, 50)
    plt.xlim(-1, 6.5)
    plt.ylim(-1, 5.5)
    plot_topo(node_list,edge_list, ax, if_arrow=False)
    trajbuffer = TM.get_traj_buffer()
    plot_cars(trajbuffer, car_length, car_width)
    plt.pause(1)
    # plt.show()


    """ 
    Car state update
    """
    
    car_states=[]
    Tbuffer = TM.get_traj_buffer()
    if len(T_history) > 0:
        for car_id in range(N_CAR):
            print(" car_id {}, old buffer {} , new {} ".format(car_id, T_history[-1][car_id][1], Tbuffer[car_id][1] ))
            # assert Tbuffer[car_id][1][:] != T_history[-1][car_id][1].all(), "No new traj planned"
    T_history.append(Tbuffer)
    # print("Sbuffer: ", TM.Sbuffer)
    for id in range(N_CAR):
        print("car ", id, " : ***************************88")    
        print("state buffer: ", TM.StateBuffer[id])
    # print("Path buffer: ", TM.PathBuffer)
    # print(" Tbuffer: ",Tbuffer)
    # print("waypoints buffer", TM.WptsBuffer)
    # assert 1==2, "check buffer."
    for id in range(N_CAR):
        print(" Tbuffer[id]: ", Tbuffer[id][1])
        car_states.append(Tbuffer[id][1])

    # for i in range(N_CAR):
        # print("Traj buffer: ",Tbuffer[i])

