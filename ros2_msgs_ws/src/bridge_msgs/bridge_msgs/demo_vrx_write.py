#!/usr/bin/env python3

# This demonstrates publishing motor commands in ROS 2 to a robot in ROS 1

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class VRXWrite(Node):

  def __init__(self):
    super().__init__('demo_vrx_write')
 
    # Publisher for thrust
    #lateral_cmd_pub = self.create_publisher(Float32,
    #  '/wamv/thrusters/lateral_thrust_cmd')
    self.left_cmd_pub = self.create_publisher(Float32,
      '/wamv/thrusters/left_thrust_cmd', 10)
    #right_cmd_pub = self.create_publisher(Float32,
    #  '/wamv/thrusters/right_thrust_cmd')

    # Callback period in seconds
    timer_period = 0.5
    self.timer = self.create_timer(timer_period, self.cmd_pub)

  def cmd_pub(self):
    cmd_msg = Float32()
    cmd_msg.data = 1.0
    self.left_cmd_pub.publish(cmd_msg)
    self.get_logger().info('Published command %g' % cmd_msg.data)


def main():
  rclpy.init(args=sys.argv)

  node = VRXWrite()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  sys.exit(main())
