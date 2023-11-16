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

        self.path = Path()
        self.path.header.frame_id = "map"  # Adjust frame ID as needed

        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndt_pose_callback)
        rospy.Subscriber('/vel_raw', Twist, self.vel_raw_callback)

        self.filtered_pose_pub = rospy.Publisher('/filtered_ndt_pose', PoseStamped, queue_size=10)
        self.filtered_path_pub = rospy.Publisher('/filtered_path', Path, queue_size=10)

    def ndt_pose_callback(self, msg):
        # Extract position and orientation
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Update state estimate with position and orientation
        self.state_estimate[0:3] = [x, y, theta]

        # Call update step of Kalman Filter (to be implemented)
        # measurement_covariance = np.diag([0.1, 0.1, np.radians(5)])  # Example values
        # self.update_step(self.state_estimate[0:3], measurement_covariance)

    def vel_raw_callback(self, msg):
        # Extract velocities
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.linear.z  # Angular velocity

        # Update state estimate with velocities
        self.state_estimate[3:] = [vx, vy, omega]

        # Call prediction step of Kalman Filter (to be implemented)
        # dt = 0.1  # Time interval, adjust as needed
        # self.prediction_step(dt)

    # def prediction_step(self, dt):
    #     x, y, theta, vx, vy, omega = self.state_estimate
    #     self.state_estimate[0] += vx * dt
    #     self.state_estimate[1] += vy * dt
    #     self.state_estimate[2] += omega * dt

    #     # Process noise covariance matrix
    #     Q = np.diag([0.1, 0.1, np.radians(5), 0.1, 0.1, np.radians(5)])
    #     self.covariance_matrix += Q


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
            self.prediction_step(dt=0.1)

            measurement_covariance = np.diag([0.1, 0.1, np.radians(5)])  # Example values
            self.update_step(self.state_estimate[0:3], measurement_covariance)
            filtered_pose_msg = PoseStamped()
            filtered_pose_msg.header.stamp = rospy.Time.now()
            filtered_pose_msg.header.frame_id = "map"
            filtered_pose_msg.pose.position.x = self.state_estimate[0]
            filtered_pose_msg.pose.position.y = self.state_estimate[1]
            filtered_pose_msg.pose.position.z = 0

            q = quaternion_from_euler(0, 0, self.state_estimate[2])
            filtered_pose_msg.pose.orientation.x = q[0]
            filtered_pose_msg.pose.orientation.y = q[1]
            filtered_pose_msg.pose.orientation.z = q[2]
            filtered_pose_msg.pose.orientation.w = q[3]

            self.filtered_pose_pub.publish(filtered_pose_msg)

            # Update and publish the path
            self.path.header.stamp = rospy.Time.now()
            self.path.poses.append(filtered_pose_msg)
            self.filtered_path_pub.publish(self.path)
            self.state_buffer.append(self.state_estimate.copy())

            rate.sleep()

if __name__ == '__main__':
    try:
        node = KalmanFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

