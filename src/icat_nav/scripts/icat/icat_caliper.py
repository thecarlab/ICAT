import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from topo import *
from math import atan2
from nav_gym.obj.geometry.util import topi
import copy

class IcatCaliper:
    def __init__(self):
        self.node_list = load_edges('/home/tian/icat_nodes.json')
        self.edge_list = load_edges('/home/tian/icat_edges.json')
        self.G = build_graph(self.node_list, self.edge_list)
        self.path = self.get_track_path()
        self.bag_file_path = '/home/tian/icat_ws/src/icat_nav/data/carto_pose.bag'
        self.pts_path = '/home/tian/icat_ws/src/icat_nav/data/pose_pts.npy'
        
        self.icat_pts = self.get_icat_points()
        self.pose_pts = self.get_record_points()
        # Get the transformation matrix
        self.transformation_matrix = self.guess_tf()

        # Transform ICAT points
        self.tf_icat_pts = self.transform_icat_points()
        self.tf_node_pts = []

        # self.downsampled_pose_pts()
        
        # Set up the matplotlib figure and axes
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.new_node_list = self.calc_new_node_list()
        tmp_new_node_list = copy.deepcopy(self.new_node_list)
        self.new_edge_list = self.calc_new_edge_list(tmp_new_node_list)
        self.new_edge_pts = self.get_new_edge_pts()

    def get_track_path(self):
        track_path = []
        path, _ = A_star_path(self.G, 1, 10)
        path.append(path[0])
        return np.array(path)

    def get_icat_points(self):
        pts = get_points_from_path(self.path, self.G)
        pts = np.array(pts)[:,:2]  # Remove the angles
        return pts

    def get_record_points(self):
        pts = np.load(self.pts_path)
        return pts
        
    def guess_tf(self):
        """
        [ cos(θ)  -sin(θ)  tx ]
        [ sin(θ)   cos(θ)  ty ]
        [   0        0     1  ]
        """
        theta = 0.04
        tx = -1.08
        ty = 0.410
        transformation_matrix = np.array([[np.cos(theta), -np.sin(theta), tx],
                                          [np.sin(theta), np.cos(theta), ty],
                                          [0, 0, 1]])

        return transformation_matrix

    def transform_icat_points(self):
        # Create a homogeneous representation of the ICAT points
        num_pts = self.icat_pts.shape[0]
        icat_homogeneous = np.hstack((self.icat_pts, np.ones((num_pts, 1))))
        
        # Apply the transformation
        transformed_homogeneous = np.dot(self.transformation_matrix, icat_homogeneous.T).T
        
        # Return the transformed points (2D only)
        return transformed_homogeneous[:, 0:2]


    def plot_points(self):
        # Extract x and y for icat_pts
        if len(self.icat_pts) > 0:
            plt.scatter(self.icat_pts[:, 0], self.icat_pts[:, 1], color='blue', label='Icat Points', marker='o')

        # Extract x, y, and yaw for pose_pts
        if len(self.pose_pts) > 0:
            plt.scatter(self.pose_pts[:, 0], self.pose_pts[:, 1], color='red', label='Recorded Points', marker='o')
            plt.scatter(self.tf_icat_pts[:, 0], self.tf_icat_pts[:, 1], color='green', label='Transformed Points', marker='o')
        
        if len(self.new_edge_pts) > 0:
            plt.scatter(self.new_edge_pts[:, 0], self.new_edge_pts[:, 1], color='orange', label='New Edge Points', marker='o')  
        
        # if len(self.tf_node_pts) > 0:
        #     plt.scatter(self.tf_node_pts[:, 0], self.tf_node_pts[:, 1], color='purple', label='New Node Points', marker='o')    
        # Additional Plot Formatting
        plt.title('Icat Points and Robot Pose Points')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.axhline(0, color='grey', lw=0.5, ls='--')
        plt.axvline(0, color='grey', lw=0.5, ls='--')
        plt.grid()
        plt.legend()
        plt.axis('equal')
        plt.show()


    def calc_tf_pts(self, pts):
        num_pts = pts.shape[0]
        icat_homogeneous = np.hstack((pts, np.ones((num_pts, 1))))
        transformed_homogeneous = np.dot(self.transformation_matrix, icat_homogeneous.T).T[:, 0:2]
        
        return transformed_homogeneous


    def calc_new_node_list(self):
        new_node_list = copy.deepcopy(self.node_list)
        node_pts, _ = get_points_from_nodes(self.node_list)


        

        tf_pts = self.calc_tf_pts(node_pts)
        
        self.tf_node_pts = tf_pts 


        # print("new_node_list: ", new_node_list)
        # print("Node pts: ", node_pts)
        # print("tf_pts: ", tf_pts)
        # print("tf_pts: ", tf_pts.shape)
        # assert 1==2, "stop"


        dx, dy = tf_pts[1] - tf_pts[0]
        theta = atan2(dy,dx)
        for i in range(len(new_node_list)):
            _, _, yaw = new_node_list[i][1]["coord"]
            x,y = tf_pts[i]
            new_node_list[i][1]["coord"] = (x,y, topi(yaw+ theta))
        print("New node list: ", new_node_list)
        # assert 1==2, "stop"
        return new_node_list

    def calc_new_edge_list(self, new_node_list):

        new_edge_list = get_edge_list(new_node_list,interval=0.05)
        # print("new_edge_list: ", new_edge_list) 
        # assert 1==2,"stop"
        return new_edge_list
    
    def save_new_nodes_edges(self):
        print("Saving new nodes and edges")
        new_node_list = self.calc_new_node_list()
        new_edge_list = self.calc_new_edge_list(new_node_list)
        print("New node list: ", new_edge_list)
        save_edges('/home/tian/icat_ws/src/icat_nav/data/carto_icat_nodes.json', new_node_list)
        save_edges('/home/tian/icat_ws/src/icat_nav/data/carto_icat_edges.json', new_edge_list)
        self.new_node_list = new_node_list
        self.new_edge_list = new_edge_list
        print("New nodes and edges saved successfully")

    def get_new_edge_pts(self):
        assert len(self.new_edge_list) > 0, "New edge list is empty"
        edge_pts, _ = get_points_from_edges(self.new_edge_list)
        return edge_pts

# Example usage
if __name__ == '__main__':
    icat_caliper = IcatCaliper()

    # icat_caliper.plot_points()
    icat_caliper.save_new_nodes_edges()
    icat_caliper.plot_points()












# from topo import *
# import networkx as nx
# import rosbag
# import numpy as np
# import tf.transformations
# from geometry_msgs.msg import PoseStamped
# # Need a map to calibrate
# # Goal: get the transformation from the icat to the lidar map
# # 1. Define the track points on the icat map
# # 2. Define the track points on the lidar map
# # 3. Use the track points to get the transformation matrix

# # 1. Define the track points on the icat map:
# class IcatCaliper:
#     def __init__(self):
#         self.node_list = load_edges('/home/tian/icat_nodes.json')
#         self.edge_list = load_edges('/home/tian/icat_edges.json')
#         # self.track_points = self.get_track_points()

#         self.G = build_graph(self.node_list, self.edge_list)
#         self.path = self.get_track_path()
#         self.bag_file_path = '/home/tian/icat_ws/src/icat_nav/data/carto_pose.bag'
        
#         self.icat_pts = self.get_icat_points()
#         self.pose_pts = self.extract_robot_pose()

#         self.downsampled_pose_pts()

#     def get_track_path(self):
#         track_path = []
#         path, _  = A_star_path(self.G,1,10)
#         path.append(path[0])
#         return np.array(path)

#     def get_icat_points(self):
#         pts = get_points_from_path(self.path, self.G)
#         pts = np.array(pts) # remove the angles
#         # print(pts)
#         # print(pts.shape)
#         return pts

#     def extract_robot_pose(self):
#         print("Extracting robot pose from the bag file")
#         pts = []
#         with rosbag.Bag(self.bag_file_path, 'r') as bag:
#             for topic, msg, t in bag.read_messages(topics=['/robot_pose']):
#                 # Ensure the message is of the correct type
#                 # print("msg: ", msg)
#                 # print("topic: ", topic)
#                 # print("t: ", t)
#                 # if isinstance(msg, PoseStamped):
#                     # Extract position (x, y)
#                 x = msg.pose.position.x
#                 y = msg.pose.position.y
#                 # Extract the quaternion
#                 quaternion = (
#                     msg.pose.orientation.x,
#                     msg.pose.orientation.y,
#                     msg.pose.orientation.z,
#                     msg.pose.orientation.w
#                 )              
#                 # Convert quaternion to Euler angles (roll, pitch, yaw)
#                 euler_angles = tf.transformations.euler_from_quaternion(quaternion)    
#                 # Yaw is the third angle
#                 yaw = euler_angles[2]
#                 # Store the extracted values
#                 pts.append(np.array([x, y,yaw]))
                
#                 # print(pts)
#                 assert len(pts) > 0, "No points extracted from the bag file"
#         return np.array(pts)

#     def downsampled_pose_pts(self):
#         # Running a clustering algorithm to cluster the points that are too close

#     def plot_points(self):
#         # Plotting icat_pts and pose_pts
#         plt.figure(figsize=(10, 8))

#         # Extract x and y for icat_pts
#         if len(self.icat_pts) > 0:
#             plt.scatter(self.icat_pts[:, 0], self.icat_pts[:, 1], color='blue', label='Icat Points', marker='o')

#         # Extract x, y, and yaw for pose_pts

#         plt.scatter(self.pose_pts[:, 0], self.pose_pts[:, 1], color='red', label='Recorded Points', marker='o')

#             # plt.quiver(self.pose_pts[:, 0], self.pose_pts[:, 1], 
#             #            np.cos(self.pose_pts[:, 2]), np.sin(self.pose_pts[:, 2]), 
#             #            angles='xy', scale_units='xy', scale=0.1, color='red', label='Pose Points')

#         # Additional Plot Formatting
#         plt.title('Icat Points and Robot Pose Points')
#         plt.xlabel('X Coordinate')
#         plt.ylabel('Y Coordinate')
#         plt.axhline(0, color='grey', lw=0.5, ls='--')
#         plt.axvline(0, color='grey', lw=0.5, ls='--')
#         plt.grid()
#         plt.legend()
#         plt.axis('equal')  # Equal aspect ratio
#         plt.show()

# if __name__ == '__main__':
#     icat_caliper = IcatCaliper()
#     print(icat_caliper.path)
#     icat_caliper.get_icat_points()
#     icat_caliper.plot_points()