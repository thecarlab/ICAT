#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque

class KalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_node')
        
        self.state_estimate = np.zeros(6)  # [x, y, theta, vx, vy, omega]
        self.covariance_matrix = np.diag([0.1, 0.1, np.radians(5), 0.1, 0.1, np.radians(5)])
        self.state_buffer = deque(maxlen=100)  # Buffer for past states, adjust size as needed
        self.track_buffer = []
        self.path = Path()
        self.path.header.frame_id = "map"  # Adjust frame ID as needed

        rospy.Subscriber('robot1/ndt_pose', PoseStamped, self.ndt_pose_callback1)
        # rospy.Subscriber('robot1/vel_raw', Twist, self.vel_raw_callback)

        rospy.Subscriber('robot2/ndt_pose', PoseStamped, self.ndt_pose_callback2)
        # rospy.Subscriber('robot2/vel_raw', Twist, self.vel_raw_callback)

        rospy.Subscriber('robot3/ndt_pose', PoseStamped, self.ndt_pose_callback3)
        # rospy.Subscriber('robot3/vel_raw', Twist, self.vel_raw_callback)

        rospy.Subscriber('robot4/ndt_pose', PoseStamped, self.ndt_pose_callback4)
        # rospy.Subscriber('robot4/vel_raw', Twist, self.vel_raw_callback)

        rospy.Subscriber('robot5/ndt_pose', PoseStamped, self.ndt_pose_callback5)
        # rospy.Subscriber('robot5/vel_raw', Twist, self.vel_raw_callback)


        rospy.Subscriber('robot6/ndt_pose', PoseStamped, self.ndt_pose_callback6)
        # rospy.Subscriber('robot6/vel_raw', Twist, self.vel_raw_callback)

        # self.filtered_pose_pub = rospy.Publisher('/filtered_ndt_pose', PoseStamped, queue_size=10)
        # self.filtered_path_pub = rospy.Publisher('/filtered_path', Path, queue_size=10)
        self.car_state_pub1 = rospy.Publisher('robot1/car_state',Twist, queue_size=10 )
        self.car_state_pub2 = rospy.Publisher('robot2/car_state',Twist, queue_size=10 )
        self.car_state_pub3 = rospy.Publisher('robot3/car_state',Twist, queue_size=10 )
        self.car_state_pub4 = rospy.Publisher('robot4/car_state',Twist, queue_size=10 )
        self.car_state_pub5 = rospy.Publisher('robot5/car_state',Twist, queue_size=10 )
        self.car_state_pub6 = rospy.Publisher('robot6/car_state',Twist, queue_size=10 )
        # self.car_state_pub = rospy.Publisher('/car_state',Twist, queue_size=10 )


    def make_twist(self, car_state):
        x,y,z = car_state
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        cmd.linear.z = z
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0

        return cmd

    def ndt_pose_callback1(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.state_estimate[0:3] = [x, y, theta]
        twist_state = self.make_twist([x,y,theta])
        self.car_state_pub1.publish(twist_state)

    def ndt_pose_callback2(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.state_estimate[0:3] = [x, y, theta]
        twist_state = self.make_twist([x,y,theta])
        self.car_state_pub2.publish(twist_state)

    def ndt_pose_callback3(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.state_estimate[0:3] = [x, y, theta]
        twist_state = self.make_twist([x,y,theta])
        self.car_state_pub3.publish(twist_state)

    def ndt_pose_callback4(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.state_estimate[0:3] = [x, y, theta]
        twist_state = self.make_twist([x,y,theta])
        self.car_state_pub4.publish(twist_state)

    def ndt_pose_callback5(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.state_estimate[0:3] = [x, y, theta]
        twist_state = self.make_twist([x,y,theta])
        self.car_state_pub5.publish(twist_state)

    def ndt_pose_callback6(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.state_estimate[0:3] = [x, y, theta]
        twist_state = self.make_twist([x,y,theta])
        self.car_state_pub6.publish(twist_state)






    # def vel_raw_callback(self, msg):
    #     # Extract velocities
    #     vx = msg.linear.x
    #     vy = msg.linear.y
    #     omega = msg.linear.z  # Angular velocity
    #     # Update state estimate with velocities
    #     self.state_estimate[3:] = [vx, vy, omega]




    def prediction_step(self, dt):
        # Use a simple average of past states for prediction if buffer has enough entries
        if len(self.state_buffer) >= 5:  # Example buffer size to consider
            last_states = np.array(list(self.state_buffer)[-5:])  # Last 5 states
            average_state = np.mean(last_states, axis=0)
            vx, vy, omega = average_state[3:6]  # Average velocity
        else:
            vx, vy, omega = self.state_estimate[3:6]

        # Predict the next state based on the average velocity
        self.state_estimate[0] += vx * dt
        self.state_estimate[1] += vy * dt
        self.state_estimate[2] += omega * dt

    def update_step(self, measurement, measurement_covariance):
        Y = measurement - self.state_estimate[0:3]  # Measurement residual
        S = self.covariance_matrix[0:3, 0:3] + measurement_covariance  # Residual covariance
        K = np.dot(self.covariance_matrix[0:3, 0:3], np.linalg.inv(S))  # Kalman gain

        self.state_estimate[0:3] += np.dot(K, Y)
        self.covariance_matrix[0:3, 0:3] = np.dot((np.eye(3) - K), self.covariance_matrix[0:3, 0:3])

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        while not rospy.is_shutdown():
            # Publish the filtered pose

            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = KalmanFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

