#!/usr/bin/env python

# Converted to ROS 1 from
#   https://index.ros.org/doc/ros2/Tutorials/Rosbag-with-ROS1-Bridge/

import sys
import time

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

def main():
    rospy.init_node('ros1_emulator', anonymous=True)

    imu_pub = rospy.Publisher('imu_data', Imu, queue_size=5)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
    cmd_pub = rospy.Publisher('cmd_vel', TwistStamped, queue_size=5)

    imu_msg = Imu()
    odom_msg = Odometry()
    cmd_msg = TwistStamped()

    rate = rospy.Rate(10)

    counter = 0
    while not rospy.is_shutdown():
        counter += 1
        now = time.time()
        if (counter % 50) == 0:
            odom_msg.header.stamp = rospy.Time.now()
            odom_pub.publish(odom_msg)

            cmd_msg.header.stamp = rospy.Time.now()
            cmd_pub.publish(cmd_msg)

            print ('published odom and cmd_vel')
        if (counter % 100) == 0:
            imu_msg.header.stamp = rospy.Time.now()
            imu_pub.publish(imu_msg)
            counter = 0
            print ('published imu')
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
