#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from topo import load_edges



class CarSimulationNode:
    def __init__(self):
        """
        This simulate a car with a simple kinematic model
        """
        rospy.init_node('car_simulation_node')
        self.dt = 0.1

        # State [x, y, theta, v, omega] inital node 4
        self.state = np.zeros(5)
        self.node_list = load_edges('/home/tian/icat_nodes.json')
        self.node_id = rospy.get_param('start_node_id', 4) 
        print("Node id: ", self.node_id)   
        self.state[:3] = self.node_list[self.node_id-1][1]["coord"][:3]

        print("Initial state: ", self.state)

        # self.wpt_dist = 0.05
        # self.dt = 0.1
        # self.n_car = 2
        # self.n_node = 42
        # self.car_param = get_car_param()
        # self.car_info = {"hl":0.1775, "hw": 0.10, "amax":0.3, "amin":-0.3, "jerkmax": 1.0}
        # self.car_states = [np.zeros(5) for i in range(self.n_car)]
        # self.node_list = load_edges('/home/tian/icat_nodes.json')
        # self.edge_list = load_edges('/home/tian/icat_edges.json')
        # self.G = build_graph(self.node_list, self.edge_list)
        # self.start_nodes, self.goal_nodes = self.sample_sg_nodes() 
        # # only for test
        # self.start_nodes = [4,5,9,29, 39,38][:self.n_car]
        # self.goal_nodes = [13,20,20,17, 16, 40][:self.n_car]

        rospy.Subscriber('cmd_vel',Twist, self.car_cmd_callback1)
        self.car_state_pub = rospy.Publisher('car_state',Twist,queue_size=10)
        self.vel_raw_pub = rospy.Publisher('vel_raw',Twist,queue_size=10)



    def calc_yaw(self,q):
        rotation = R.from_quat(q)
        euler_angles = rotation.as_euler('zyx', degrees = True)
        yaw = euler_angles[0]
        # print("Yaw angle: ", yaw_angle, "degrees")
        return yaw



    def car_cmd_callback1(self, msg):
        # Extract velocities
        cmd_v = msg.linear.x
        cmd_w = msg.angular.z

        self.state[0] = self.state[0] + cmd_v * np.cos(self.state[2]) * self.dt
        self.state[1] = self.state[1] + cmd_v * np.sin(self.state[2]) * self.dt
        self.state[2] = self.state[2] + cmd_w * self.dt
        self.state[3] = cmd_v
        self.state[4] = cmd_w

        # car_state_msg = Twist()
        # car_state_msg.linear.x = self.state[0]
        # car_state_msg.linear.y = self.state[1]
        # car_state_msg.linear.z = self.state[2]
        # self.car_state_pub.publish(car_state_msg)

        # vel_raw_msg = Twist()
        # vel_raw_msg.linear.x = self.state[3]
        # vel_raw_msg.linear.y = 0
        # vel_raw_msg.linear.z = self.state[4]
        # self.vel_raw_pub.publish(vel_raw_msg)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # Your simulation code here
            car_state_msg = Twist()
            car_state_msg.linear.x = self.state[0]
            car_state_msg.linear.y = self.state[1]
            car_state_msg.linear.z = self.state[2]
            self.car_state_pub.publish(car_state_msg)

            vel_raw_msg = Twist()
            vel_raw_msg.linear.x = self.state[3]
            vel_raw_msg.linear.y = 0
            vel_raw_msg.linear.z = self.state[4]
            self.vel_raw_pub.publish(vel_raw_msg)
            rate.sleep()




if __name__ == '__main__':
    try:
        car_simulation_node = CarSimulationNode()
        car_simulation_node.run()
    except rospy.ROSInterruptException:
        pass

