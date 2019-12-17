#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def destination_publisher():
    pub = rospy.Publisher('Destination', Float64MultiArray, queue_size=10)
    rospy.init_node('Destinationxyz', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    

    while not rospy.is_shutdown():
        xyz_pub = Float64MultiArray()
        xyz_pub.data = [0,0.5,.5]

        # rospy.loginfo(hello_str)
        pub.publish(xyz_pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        destination_publisher()
    except rospy.ROSInterruptException:
        pass
        