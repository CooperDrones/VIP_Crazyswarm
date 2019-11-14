#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from vicon_bridge.srv import viconGrabPose

def grabPose(vicon_object):
    rospy.wait_for_service('/vicon/grab_vicon_pose')
    pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)
    pose = pose_getter(vicon_object, vicon_object, 1)
    return pose.pose.pose.position

if __name__ == "__main__":
    
    rospy.init_node('plotter')
    rate = rospy.Rate(30)

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # for i in range(10)
    while not rospy.is_shutdown():
        pose = grabPose('crazyflie3')
        ax.scatter3D(pose.x, pose.y, pose.z)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        rate.sleep()
        plt.ion()
        plt.show(block=False)
        plt.pause(0.01)