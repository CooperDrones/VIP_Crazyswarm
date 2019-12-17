#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def callback(intake):
    rospy.loginfo(intake.data)

def destination_listener():
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("Destination", Float64MultiArray,callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    destination_listener()