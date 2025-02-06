#!/usr/bin/env python3

import rospy
import numpy as np
import casadi as ca
from geometry_msgs.msg import Twist

import rospy

from nav_msgs.msg import Path


class MPC:
    def __init__(self):

        self.Tx = []
        self.Ty = []
        self.x = None
        self.y = None
        self.theta = None
        self.v = None
        self.w = None

        self.N = 20
        self.dt = 0.1
        self.opti = ca.Opti()


        self.traj_path_sub = rospy.Subscriber("/robot1/traj_path", Path, self.traj_path_callback)
        self.car_state_sub = rospy.Subscriber("/robot1/car_state", Twist, self.car_state_callback)
        self.vel_raw_sub = rospy.Subscriber("/robot1/vel_raw", Twist, self.vel_raw_callback)

        self.control_pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)

    def traj_path_callback(self, msg):
        self.Tx = [p.pose.position.x for p in msg.poses]
        self.Ty = [p.pose.position.y for p in msg.poses]
        # print(self.Tx, self.Ty)

    def car_state_callback(self, msg):
        self.x = msg.linear.x
        self.y = msg.linear.y
        self.theta = msg.linear.z

        print("Car state: ", self.x, self.y, self.theta)

    def vel_raw_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

        print("Raw velocities: ", self.v, self.w)

    def QP_method(self):
        # Ensure initial conditions are set
        if self.x is None or self.y is None or self.theta is None:
            rospy.logwarn("Car state not received yet.")
            return

        print("QP method")

        horizon = min(self.N, len(self.Tx))
        if horizon <= 1:
            rospy.logwarn("Not enough points in trajectory path.")
            return

        # Reset the Opti instance for each QP call
        self.opti = ca.Opti()

        # Model parameters
        x = self.opti.variable(horizon)
        y = self.opti.variable(horizon)
        theta = self.opti.variable(horizon)

        v = self.opti.variable(horizon-1)
        omega = self.opti.variable(horizon-1)

        # Setup the objective function
        cost = 0
        for i in range(horizon-1):
            cost += (x[i] - self.Tx[i]) ** 2 + (y[i] - self.Ty[i]) ** 2  # Tracking cost
            cost += 0.001* v[i]**2 + 0.01* omega[i]**2  # Control effort cost

        self.opti.minimize(cost)

        # Set the constraints
        for i in range(horizon-1):
            next_x = x[i] + v[i] * ca.cos(theta[i]) * self.dt
            next_y = y[i] + v[i] * ca.sin(theta[i]) * self.dt
            next_theta = theta[i] + omega[i] * self.dt

            self.opti.subject_to(x[i+1] == next_x)
            self.opti.subject_to(y[i+1] == next_y)
            self.opti.subject_to(theta[i+1] == next_theta)

        # Initial conditions
        self.opti.subject_to(x[0] == self.x)
        self.opti.subject_to(y[0] == self.y)
        self.opti.subject_to(theta[0] == self.theta)

        # Dynamic bounds
        vmax = 0.2  # Maximum linear velocity
        vmin = 0.01
        omegamax = 0.8  # Maximum angular velocity
        self.opti.subject_to(self.opti.bounded(vmin, v, vmax))
        self.opti.subject_to(self.opti.bounded(-omegamax, omega, omegamax))

        # Solver options
        opts = {"ipopt.print_level": 0, "print_time": 0, "expand": True}
        self.opti.solver("ipopt", opts)

        # Solve the optimization problem
        try:
            print("try to solve!")
            sol = self.opti.solve()
            optimal_v = sol.value(v[0])
            optimal_omega = sol.value(omega[0])
            print("Optimal v: ", optimal_v)
            print("Optimal omega: ", optimal_omega)

            # Publish the control command
            cmd_vel = Twist()
            cmd_vel.linear.x = optimal_v
            cmd_vel.angular.z = optimal_omega
            self.control_pub.publish(cmd_vel)
        except Exception as e:
            rospy.logwarn("QP method failed to find a solution: %s", str(e))

    def run(self):
        print("Run")
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():

            self.QP_method()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mpc_node', anonymous=True)
    mpc = MPC()
    mpc.run()