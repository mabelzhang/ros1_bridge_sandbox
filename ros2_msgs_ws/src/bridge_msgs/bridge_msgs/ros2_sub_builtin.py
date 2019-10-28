#!/usr/bin/env python3

# From https://index.ros.org/doc/ros2/Tutorials/Rosbag-with-ROS1-Bridge/

import sys
import time
import random

import rclpy

from std_msgs.msg import Float32

def callback(msg):
    print('received floating_point %f' % msg.data)

def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('ros2_sub_builtin')

    fp_sub = node.create_subscription(Float32, 'floating_point', callback, 10)
    fp_sub
    print('subscribing to floating_point')

    while rclpy.ok():
        rclpy.spin_once(node)

        time.sleep(0.001)

if __name__ == '__main__':
    sys.exit(main())
