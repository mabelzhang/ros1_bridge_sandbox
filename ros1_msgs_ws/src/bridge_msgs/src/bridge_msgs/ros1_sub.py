#!/usr/bin/env python

import sys
import time

import rospy

from bridge_msgs.msg import JointCommand


def callback(data):
    rospy.loginfo('%s' % data.position)

def main():
    rospy.init_node('ros1_sub', anonymous=True)

    rospy.Subscriber('joint_command', JointCommand, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
