#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(msg):
    print(msg.data)

rospy.init_node('ultrasonic_sub')
sub1 = rospy.Subscriber('right_ultrasonic', Int32, callback)
sub2 = rospy.Subscriber('left_ultrasonic', Int32, callback)

rospy.spin()
