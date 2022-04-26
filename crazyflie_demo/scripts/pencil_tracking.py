#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.srv import Takeoff,Land
from vicon_bridge.srv import viconGrabPose

class Demo():
    def __init__(self, offset_x, offset_y, prefix):
        rospy.init_node('demo_cooper', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        self.goals = [[0, 0, 0.4, 0, 0]]
        self.goalIndex = 0
        self.offset_x = offset_x
        self.offset_y = offset_y
        # rospy.wait_for_service(prefix + '/takeoff')
        # self.takeoff_request = rospy.ServiceProxy(prefix + '/takeoff', Takeoff)
        # rospy.wait_for_service(prefix + '/land')
        # self.land_request = rospy.ServiceProxy(prefix + '/takeoff', Land)
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)
        self.record_rate = rospy.Rate(2) # every second
        self.time_delay = rospy.Rate(0.1) # 10 seconds

    def getPose(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        self.position = self.pose.pose.pose.position
        self.position_list = [self.position.x, self.position.y -0.7, self.position.z]
        return self.position_list
    
    # def recordPencil(self, t):
    #     self.time_delay.sleep()
    #     self.yaw = 0
    #     self.time = 0.5
    #     for i in range(t):
    #         self.pencil_goal = self.getPose('pencil')
    #         self.pencil_goal.append(self.yaw)
    #         self.pencil_goal.append(self.time)
    #         self.goals.append(self.pencil_goal)
    #         self.record_rate.sleep()
    #     print(self.goals)
    #     return self.goals

    def run(self):
        self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
        goal = PoseStamped()
        goal.header.seq = 0
        goal.header.frame_id = self.worldFrame
        self.counter = 0
        # self.takeoff_request()
        while not rospy.is_shutdown():
            goal.header.seq += 1
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = self.goals[self.goalIndex][0]
            goal.pose.position.y = self.goals[self.goalIndex][1]
            goal.pose.position.z = self.goals[self.goalIndex][2]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            
            if self.counter >= 200:
                self.pencil_pose = self.getPose('pencil')
                print(self.pencil_pose)
                goal.pose.position.x = self.pencil_pose[0] + self.offset_x
                goal.pose.position.y = self.pencil_pose[1] - 0.5 + self.offset_y
                goal.pose.position.z = self.pencil_pose[2]

            self.pubGoal.publish(goal)
            self.counter += 1

# if __name__ == '__main__':
#     cf3_pencil_tracker = Demo(0.25, '/crazyflie3')
#     # cf3_pencil_tracker.recordPencil(10) # record for 10 iterations
#     cf3_pencil_tracker.run()

#     # cf4_pencil_tracker = Demo(-0.25, '/crazyflie4')
#     # cf4_pencil_tracker.run()

#   while not rospy.is_shutdown():
#             goal.header.seq += 1
#             goal.header.stamp = rospy.Time.now()
#             goal.pose.position.x = self.goals[self.goalIndex][0]
#             goal.pose.position.y = self.goals[self.goalIndex][1]
#             goal.pose.position.z = self.goals[self.goalIndex][2]
#             quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
#             goal.pose.orientation.x = quaternion[0]
#             goal.pose.orientation.y = quaternion[1]
#             goal.pose.orientation.z = quaternion[2]
#             goal.pose.orientation.w = quaternion[3]
            
#             if self.counter >= 200:
#                 self.pencil_pose = self.getPose('pencil')
#                 print(self.pencil_pose)
#                 goal.pose.position.x = self.pencil_pose[0] + self.offset_x
#                 goal.pose.position.y = self.pencil_pose[1] - 0.5 + self.offset_y
#                 goal.pose.position.z = self.pencil_pose[2]

#             self.pubGoal.publish(goal)
#             self.counter += 1