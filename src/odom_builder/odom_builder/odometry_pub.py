import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry
# from transformations import quaternion_from_euler

import math


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class Encoder_Subscriber(Node):
    def __init__(self):
        super().__init__('odometry_pub')
        self.subscription = self.create_subscription(String,'/micro_ros_arduino_node_publisher',self.listener_callback,10)
        self.subscription
        self.first_callback = True
        self.left_prev = 0
        self.right_prev = 0
        self.gear_reduction = 784.0 / 81.0
        self.wheel_circum = .0905 * np.pi
        self.wheel_base = .201707-.01
        self.dist_const = self.wheel_circum / (512*4*self.gear_reduction)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom_pub = self.create_publisher(Odometry, "/odom", 1)

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = now
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0,0, self.theta)
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        self.odom_pub.publish(odom)

    def listener_callback(self, msg):
        data = msg.data.split(" ")
        new_left = int(data[0])
        new_right = int(data[1])
        # print("left:",new_left,"right:",new_right)
        if self.first_callback:
            self.first_callback = False
            self.left_prev = new_left
            self.right_prev = new_right
            return
        delta_r = (new_right -self.right_prev) * self.dist_const
        delta_l = (new_left - self.left_prev) * self.dist_const
        delta_s = (delta_r + delta_l) / 2
        delta_theta = (delta_r - delta_l) / (self.wheel_base)
        self.x += delta_s*np.cos(self.theta+delta_theta)
        self.y += delta_s*np.sin(self.theta+delta_theta)
        self.theta += delta_theta
        self.theta %= 2*np.pi
        print(self.x, self.y, self.theta)
        self.left_prev = new_left
        self.right_prev = new_right
        self.publish_odometry()






def main(args=None):
    rclpy.init(args=args)
    encoder_subscriber = Encoder_Subscriber()
    rclpy.spin(encoder_subscriber)

    encoder_subscriber.destroy_node()
    rclpy.shutdown


if __name__ == "__main__":
    main()