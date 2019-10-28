#!/usr/bin/env python3

# From https://index.ros.org/doc/ros2/Tutorials/Rosbag-with-ROS1-Bridge/

import sys
import time
import random

import rclpy

from bridge_msgs.msg import JointCommand

def callback(msg):
    print('received joint_command %f' % msg.position)

def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('ros2_sub')

    joint_cmd_sub = node.create_subscription(JointCommand, 'joint_command',
        callback, 10)
    joint_cmd_sub
    print('subscribing to joint_command')

    while rclpy.ok():
        rclpy.spin_once(node)

        time.sleep(0.001)

if __name__ == '__main__':
    sys.exit(main())
