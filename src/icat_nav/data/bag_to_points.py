import numpy as np
import rosbag
import tf

def extract_robot_pose(bag_file_path):
    pts = []
    with rosbag.Bag(bag_file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/robot_pose']):
            
            x = msg.pose.position.x
            y = msg.pose.position.y
            quaternion = (msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w)
            euler_angles = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler_angles[2]
            pts.append(np.array([x, y, yaw]))
    return np.array(pts)

if __name__ == "__main__":
    bag_file_path = '/home/tian/icat_ws/src/icat_nav/data/carto_pose.bag'
    pts = extract_robot_pose(bag_file_path)
    np.save('/home/tian/icat_ws/src/icat_nav/data/pose_pts.npy', pts)