#!/usr/bin/env python3

# This demonstrates subscribing in ROS 2 to sensor messages published ROS 1,
#   and publishing motor commands in ROS 2 to robots in ROS 1.

import sys
import time

import rclpy
#from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32


def img_cb(msg):

  print('Received image of %d x %d at %d.%d' % (msg.height, msg.width,
    msg.header.stamp.sec, msg.header.stamp.nanosec))


def cloud_cb(msg):

  print('Received point cloud at %d.%d' % (msg.header.stamp.sec,
    msg.header.stamp.nanosec))


def main():
  rclpy.init(args=sys.argv)

  node = rclpy.create_node('demo_bridge_vrx')

  # Subscribe to camera images
  # Other camera images:
  #   /wamv/sensors/cameras/front_right_camera/image_raw
  #   /wamv/sensors/cameras/middle_right_camera/image_raw
  img_sub = node.create_subscription(Image,
    '/wamv/sensors/cameras/front_left_camera/image_raw', img_cb, 1)
  # Subscribe to LIDAR point cloud
  cloud_sub = node.create_subscription(PointCloud2,
    '/wamv/sensors/cameras/lidar_wamv/points', cloud_cb, 1)

  # Publisher for lateral thrust
  #lateral_cmd_pub = node.create_publisher(Float32,
  #  '/wamv/thrusters/lateral_thrust_cmd')
  left_cmd_pub = node.create_publisher(Float32,
    '/wamv/thrusters/left_thrust_cmd', 10)
  #right_cmd_pub = node.create_publisher(Float32,
  #  '/wamv/thrusters/right_thrust_cmd')

  while rclpy.ok():
    rclpy.spin_once(node)

    cmd_msg = Float32()
    cmd_msg.data = 1.0 # TODO
    left_cmd_pub.publish(cmd_msg)

    time.sleep(0.001)

  rclpy.shutdown()


if __name__ == '__main__':
  sys.exit(main())
