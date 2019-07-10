#!/usr/bin/env python

import sys
import time

import rospy

from std_msgs.msg import Float32


def callback(data):
    rospy.loginfo('%s' % data.data)

def main():
    rospy.init_node('ros1_sub_builtin', anonymous=True)

    rospy.Subscriber('floating_point', Float32, callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
