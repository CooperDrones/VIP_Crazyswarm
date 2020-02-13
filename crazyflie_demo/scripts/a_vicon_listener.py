#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped


class Tester():
    def __init__(self):
        rospy.init_node('vicon_listener', anonymous=True)

    def callback(self, pose):
        self.pose = pose
        # print(self.pose)
    
    def listener(self):
        rospy.Subscriber("/vicon/crazyflie4/crazyflie4", TransformStamped, self.callback)
        rospy.spin()
        pose = self.pose
        print(pose)

if __name__ == '__main__':
    test = Tester()
    test.listener()