#!/usr/bin/env python

import sys
import time
import random

import rospy

from std_msgs.msg import Float32

def main():
    rospy.init_node('ros1_pub_builtin', anonymous=True)

    fp_pub = rospy.Publisher('floating_point', Float32, queue_size=5)

    fp_msg = Float32()

    rate = rospy.Rate(10)

    counter = 0
    while not rospy.is_shutdown():
        counter += 1
        now = time.time()

        fp_msg.data = random.random()
        fp_pub.publish(fp_msg)
        print ('published floating_point %f' % fp_msg.data)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
