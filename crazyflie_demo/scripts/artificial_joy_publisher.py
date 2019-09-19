#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

rospy.init_node('a')
pub = rospy.Publisher('crazyflie/joy', Joy)
rate = rospy.Rate(2)

j = Joy()
j.buttons[3] = 1

while not rospy.is_shutdown():
    pub.publish(j)
    rate.sleep()
