#!/usr/bin/env python3

# From https://index.ros.org/doc/ros2/Tutorials/Rosbag-with-ROS1-Bridge/

import sys
import time

import rclpy

from bridge_msgs.msg import JointCommand

def main():
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('ros2_emulator')

    joint_cmd_pub = node.create_publisher(JointCommand, 'joint_command')

    joint_cmd_msg = JointCommand()
    counter = 0
    while True:
        counter += 1
        now = time.time()
        if (counter % 50) == 0:
            joint_cmd_msg.header.stamp.sec = int(now)
            joint_cmd_msg.header.stamp.nanosec = int(now * 1e9) % 1000000000
            joint_cmd_pub.publish(joint_cmd_msg)
            counter = 0
            print ('Published joint_command')
        time.sleep(0.001)


if __name__ == '__main__':
    sys.exit(main())
