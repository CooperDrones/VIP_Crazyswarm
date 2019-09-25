#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3

rospy.init_node('test')
pub = rospy.Publisher('crazyflie1/cmd_vel', Twist, queue_size=0)
rate = rospy.Rate(2)

t = Twist()
t.linear = Vector3(1, 1, 1)
t.angular = Vector3(0, 0, 0)

while not rospy.is_shutdown():
    pub.publish(t)
    rate.sleep()