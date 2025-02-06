#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    twist.linear.x = 1*data.axes[1]
    twist.linear.y = 0.045*data.axes[3]
    # twist.angular.z = 0.6*data.axes[3]

    # twist.angular.z = 1*data.axes[3]
    pub.publish(twist)



# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    # pub = rospy.Publisher("robot1/cmd_vel", Twist)
    pub = rospy.Publisher("robot1/cmd_vel", Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('js_control')
    rospy.spin()

if __name__ == '__main__':
    start()