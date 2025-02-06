#!/usr/bin/env python2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import argparse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import json
import numpy as np
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path('icat_nav')
print ("package_path: ", package_path)
parser = argparse.ArgumentParser(description="Example script")
parser.add_argument('--edgepath', type=str, default=package_path+ '/data/icat_edges.json', help='edge list path')
parser.add_argument('--nodepath', type=str, default=package_path+ '/data/icat_nodes.json', help='node list path')
# Global variables
path_publisher = None
# path = Path()
args = parser.parse_args()
print ("args: ", args)

def load_nodes(filename):
    with open(filename, 'r') as file:
        json_data = file.read()
        
    # Deserialize the JSON data
    node_list = json.loads(json_data)
    return node_list

def load_edges(filename):
    with open(filename, 'r') as file:
        json_data = file.read()
        
    # Deserialize the JSON data
    edge_list = json.loads(json_data)
    
    return edge_list

def get_points_from_edges(edge_list):
    pts = []
    theta = []
    for i, edge in enumerate(edge_list):
        for point in  edge[2]["waypoints"]:
            pts.append(np.array([point[0], point[1]]))
            theta.append(point[2])
    return np.array(pts), np.array(theta)

class IcatPathNode:
    def __init__(self):
        rospy.init_node('icat_line_path_node')
        self.path = Path()
        self.path.header.frame_id = "map"
        self.pub = rospy.Publisher('/icat_line_path', Path, queue_size=10)
        edge_list = load_edges(args.filepath)
        self.pts, self.angles = get_points_from_edges(edge_list)
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.path.header.stamp = rospy.Time.now()
            for i in range(len(self.pts)):
                point = self.pts[i]
                yaw = self.angles[i]

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = point[0]
                pose_msg.pose.position.y = point[1]
                pose_msg.pose.position.z = 0    
                q = quaternion_from_euler(0, 0, yaw)
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]    
                self.path.poses.append(pose_msg)

            self.pub.publish(self.path)
            rate.sleep()



class IcatPathNode:
    def __init__(self):
        rospy.init_node('icat_path_node')
        self.pub = rospy.Publisher('/icat_path', MarkerArray, queue_size=10)
        edge_list = load_edges(args.edgepath)
        node_list = load_nodes(args.nodepath)
        self.edges = edge_list
        self.nodes = node_list

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            marker_array = MarkerArray()
            for i, edge in enumerate(self.edges):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "icat_path"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.1  # Line width
                marker.color.a = 1.0
                marker.color.r = 1  # Red color, you can change this
                marker.color.g = 0.8
                marker.color.b = 0.0
                # marker.color.r = 1.0
                # 37,150,190
                for point in edge[2]["waypoints"]:
                    p = Point()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = -0.2
                    marker.points.append(p)
                end_node = self.nodes[edge[1]-1]
                p = Point()
                p.x = end_node[1]['coord'][0]
                p.y = end_node[1]['coord'][1]
                p.z = -0.2
                marker.points.append(p)
                marker_array.markers.append(marker)
            # Ensure all previously published markers are updated or deleted
            if len(marker_array.markers) < len(self.edges):
                for i in range(len(marker_array.markers), len(self.edges)):
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "icat_path"
                    marker.id = i
                    marker.action = Marker.DELETE
                    marker_array.markers.append(marker)
            self.pub.publish(marker_array)
            rate.sleep()


if __name__ == '__main__':
    edge_list = load_edges(args.edgepath)
    # print(edge_list)
    assert len(edge_list) > 0, "Edge list is empty"
    
    try:
        node = IcatPathNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
