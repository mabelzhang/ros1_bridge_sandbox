#!/usr/bin/env python

import sys
import time
import random

import rospy

from bridge_msgs.msg import JointCommand

def main():
    rospy.init_node('ros1_pub', anonymous=True)

    jc_pub = rospy.Publisher('joint_command', JointCommand, queue_size=5)

    jc_msg = JointCommand()

    rate = rospy.Rate(10)

    counter = 0
    while not rospy.is_shutdown():
        counter += 1
        now = time.time()
        #if (counter % 50) == 0:

        jc_msg.position = random.random()
        jc_pub.publish(jc_msg)
        print ('published joint_command %f' % jc_msg.position)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
