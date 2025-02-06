#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav_gym.obj.geometry.util import topi
from collections import deque
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
from scipy.spatial.transform import Rotation as R
import cvxpy as cp




class StopNode:
    def __init__(self):
        rospy.init_node('stop_node')

        self.n_car = 4

        self.cmd_pub1 = rospy.Publisher('robot1/cmd_vel',Twist,queue_size=10)
        self.cmd_pub2 = rospy.Publisher('robot2/cmd_vel',Twist,queue_size=10)
        self.cmd_pub3 = rospy.Publisher('robot3/cmd_vel',Twist,queue_size=10)
        self.cmd_pub4 = rospy.Publisher('robot4/cmd_vel',Twist,queue_size=10)
        self.cmd_pub5 = rospy.Publisher('robot5/cmd_vel',Twist,queue_size=10)
        self.cmd_pub6 = rospy.Publisher('robot6/cmd_vel',Twist,queue_size=10)



    def make_cmd_vel(self, raw_cmd):
        v,w = raw_cmd
        cmd = Twist()
        cmd.linear.x = v
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = w

        return cmd

    # def compute_control(self, H, traj, current_state):
    #     # State and control dimensions
    #     nx = 3  # State dimension [x, y, yaw]
    #     nu = 2  # Control dimension [v, w]
        
    #     orig_traj = traj
    #     traj = traj[:, :3].T

    #     current_state = current_state[:3]
    #     print("traj: ",traj)
    #     print("c state: ", current_state)
    #     # Control variables (velocity and angular velocity)
    #     v = cp.Variable((nu, H))

    #     # State variables (x, y, yaw)
    #     x = cp.Variable((nx, H + 1))

    #     # Cost function components
    #     cost = 0
    #     constraints = []

    #     for k in range(H):
    #         # Cost for state error
    #         state_error = cp.norm(x[:, k+1] - traj[:, k], 2)
    #         # Cost for control input
    #         control_input = cp.norm(v[:, k], 2)

    #         # cost += state_error**2 + control_input**2
    #         cost += state_error**2

    #         # Dynamics constraints (a simple kinematic model)
    #         # x_next = x_current + v_current * cos(yaw) * dt
    #         # y_next = y_current + v_current * sin(yaw) * dt
    #         # yaw_next = yaw_current + w_current * dt
    #         a_cos_theta = np.cos(orig_traj[k][2])
    #         a_sin_theta = np.sin(orig_traj[k][2])
    #         # x[0, k+1] == x[0, k] + v[0, k] * cp.cos(x[2, k]) * self.dt,
    #         # x[1, k+1] == x[1, k] + v[0, k] * cp.sin(x[2, k]) * self.dt,
    #         constraints += [

    #             x[0, k+1] == x[0, k] + v[0, k] * a_cos_theta * self.dt,
    #             x[1, k+1] == x[1, k] + v[0, k] * a_sin_theta * self.dt,
    #             x[2, k+1] == x[2, k] + v[1, k] * self.dt
    #         ]

    #         # Add any additional constraints, such as control input limits

    #     # Initial condition constraint
    #     constraints += [x[:,0] == current_state]

    #     # Formulate optimization problem
    #     problem = cp.Problem(cp.Minimize(cost), constraints)

    #     # Solve optimization problem
    #     problem.solve()

    #     # Extract first control input
    #     control_input = v[:,0].value
    #     return control_input        



    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd_list = []    
            for i in range(self.n_car):



                cmd = [0,0]


                print("robot {} final cmd : {}".format(i+1,cmd))
                cmd_list.append(cmd)
            
            for i in range(self.n_car):
                twist_cmd = self.make_cmd_vel(cmd_list[i])
                if i == 0:
                    self.cmd_pub1.publish(twist_cmd)
                elif i == 1:
                    self.cmd_pub2.publish(twist_cmd)
                elif i == 2:
                    self.cmd_pub3.publish(twist_cmd)
                elif i == 3:
                    self.cmd_pub4.publish(twist_cmd)
                elif i == 4:
                    self.cmd_pub5.publish(twist_cmd)
                elif i == 5:
                    self.cmd_pub6.publish(twist_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = StopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass