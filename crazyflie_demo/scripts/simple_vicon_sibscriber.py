#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
import inspect

def callback(msg):
    # print(msg)
    print(msg.transform.translation.x)

rospy.init_node('simple_vicon_subscriber')
sub = rospy.Subscriber('vicon/crazyflie2/crazyflie2', TransformStamped, callback)
rospy.spin()