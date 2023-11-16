#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Global variables
path_publisher = None
path = Path()

def ndt_pose_callback(msg):
    global path
    # ... [Your existing pose update logic] ...

    # Create a PoseStamped message from the msg
    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose

    # Append this pose to the path
    path.header = msg.header
    path.poses.append(pose_stamped)

    # Publish the path
    path_publisher.publish(path)

def path_node():
    global path_publisher
    rospy.init_node('path_node')
    
    rospy.Subscriber('/ndt_pose', PoseStamped, ndt_pose_callback)

    # Initialize the path publisher
    path_publisher = rospy.Publisher('/path', Path, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        path_node()
    except rospy.ROSInterruptException:
        pass
