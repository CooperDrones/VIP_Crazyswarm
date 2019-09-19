#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped,Twist,TransformStamped
# import tf2_ros
# import geometry_msgs.msg
import inspect

def callback(msg):
    # print(msg)
    print(msg.transform.translation.x)

rospy.init_node('simple_vicon_subscriber')
sub = rospy.Subscriber('vicon/crazyflie2/crazyflie2', TransformStamped, callback)
# sub = rospy.Subscriber('crazyflie/cmd_vel', Twist, callback)
# sub = rospy.Subscriber('crazyflie/goal', PoseStamped, callback)
# sub = rospy.Subscriber('tf_static', TFMessage, callback)
rospy.spin()