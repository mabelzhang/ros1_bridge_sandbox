#!/usr/bin/env python3

# This demonstrates subscribing in ROS 2 to sensor messages published ROS 1,
#   and publishing motor commands in ROS 2 to robots in ROS 1.

import sys

import rclpy
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32


def img_cb(msg):

  pass


def cloud_cb(msg):

  # TODO
  pass


def main():
  rclpy.init(args=sys.argv)

  node = rclpy.create_node('demo_bridge_vrx')

  # Subscribe to camera images
  # Other camera images: /front_right_camera/image_raw,
  #   /middle_right_camera/image_raw,
  img_sub = node.create_subscription(Image, '/front_left_camera/image_raw',
    img_cb)
  # Subscribe to LIDAR point cloud
  cloud_sub = node.create_subscription(PointCloud2, '/lidar_wamv/points',
    cloud_cb)

  # Publisher for lateral thrust
  #lateral_cmd_pub = node.create_publisher(Float32, '/lateral_thrust_cmd')
  left_cmd_pub = node.create_publisher(Float32, '/left_thrust_cmd')
  #right_cmd_pub = node.create_publisher(Float32, '/right_thrust_cmd')

  while rclpy.ok():
    rclpy.spin_once(node)

    cmd_msg = Float32()
    cmd_msg.data = 1.0 # TODO
    left_cmd_pub.publish(cmd_msg)

    time.sleep(0.001)


if __name__ == '__main__':
  sys.exit(main())
