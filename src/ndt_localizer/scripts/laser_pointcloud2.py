#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection


class LaserToPC2:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('scan_to_pointcloud2', anonymous=True)

        # Create a LaserProjection object to convert LaserScan to PointCloud2
        self.laser_projector = LaserProjection()

        # Subscribe to the topic that publishes LaserScan messages
        self.laser_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback)

        # Publish PointCloud2 messages
        self.pc2_pub = rospy.Publisher(
            '/points_raw', PointCloud2, queue_size=1)

    def scan_callback(self, data):
        # Convert LaserScan to PointCloud2
        cloud2 = self.laser_projector.projectLaser(data)

        # Publish the PointCloud2 message
        self.pc2_pub.publish(cloud2)


if __name__ == '__main__':
    try:
        ltpc2 = LaserToPC2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
