#!/usr/bin/env python3

# This demonstrates subscribing in ROS 2 to sensor messages published in ROS 1.

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu, JointState


class VRXRead(Node):

  def __init__(self):
    super().__init__('demo_vrx_read')

    # Subscribe to camera images
    # Other camera images:
    #   /wamv/sensors/cameras/front_right_camera/image_raw
    #   /wamv/sensors/cameras/middle_right_camera/image_raw
    self.img_sub = self.create_subscription(Image,
      '/wamv/sensors/cameras/front_left_camera/image_raw', self.img_cb, 1)

    # Subscribe to LIDAR point cloud
    #self.cloud_sub = self.create_subscription(PointCloud2,
    #  '/wamv/sensors/cameras/lidar_wamv/points', self.cloud_cb, 1)
 
    self.imu_sub = self.create_subscription(Imu,
      '/wamv/sensors/imu/imu/data', self.imu_cb, 10)
 
    # This is high frequency data
    #self.joint_sub = self.create_subscription(JointState,
    #  '/wamv/joint_states', self.joint_cb, 10)

  def img_cb(self, msg):

    self.get_logger().info('Received image of %d x %d at %d.%d' % (
      msg.height, msg.width, msg.header.stamp.sec, msg.header.stamp.nanosec))

  def cloud_cb(self, msg):

    self.get_logger().info('Received point cloud at %d.%d' % (
      msg.header.stamp.sec, msg.header.stamp.nanosec))

  def imu_cb(self, msg):

    self.get_logger().info('Received IMU data at %d.%d' % (msg.header.stamp.sec,
      msg.header.stamp.nanosec))

  def joint_cb(self, msg):

    self.get_logger().info('Received joint states at %d.%d' % (
      msg.header.stamp.sec, msg.header.stamp.nanosec))

def main():
  rclpy.init(args=sys.argv)

  node = VRXRead()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  sys.exit(main())
