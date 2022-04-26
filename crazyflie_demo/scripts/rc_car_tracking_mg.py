#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
import os
from tf import TransformListener
from std_srvs.srv._Empty import Empty
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from crazyflie_driver.srv import Takeoff,Land, LandRequest
from vicon_bridge.srv import viconGrabPose

class Demo():
    def __init__(self, offset_x, offset_y, offset_z, prefix):
        rospy.init_node('demo_cooper', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.listener = TransformListener()
        self.goals = [[0, 0, 0.4, 0, 0]]
        self.goalIndex = 0
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z
        self.prev_pose = [0,0,0]
        self.vd = [0,0,0]
        # rospy.wait_for_service(prefix + '/takeoff')
        # self.takeoff_request = rospy.ServiceProxy(prefix + '/takeoff', Takeoff)
        # rospy.wait_for_service(prefix + '/land')
        # self.land_request = rospy.ServiceProxy(prefix + '/takeoff', Land)
        print('wait for vicon')
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        print('got vicon')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)
        self.record_rate = rospy.Rate(2) # every second
        self.time_delay = rospy.Rate(0.1) # 10 seconds

    def getPose(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        self.position = self.pose.pose.pose.position
        self.position_list = [self.position.x, self.position.y, self.position.z]
        return self.position_list

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
            error = [0, 0, 0]
            k = 0.5
            if self.counter >= 200:
                self.car_pose = self.getPose('rc_car')
                self.drone_pose = self.getPose('crazyflie3')

                # error[0] = self.car_pose[0] - self.drone_pose[0]
                # error[1] = self.car_pose[1] - self.drone_pose[1]
                # error[2] = self.car_pose[2] - self.drone_pose[2]
                # print(error)

                # goal.pose.position.x = self.car_pose[0] - k * error[0] + self.offset_x
                # goal.pose.position.y = self.car_pose[1] - k * error[1] + self.offset_y
                # goal.pose.position.z = self.car_pose[2] + self.offset_z
                
                # self.car_pose = self.getPose('rc_car')
                # self.drone_pose = self.getPose('crazyflie3')
                # # print(self.car_pose)

                # print("carpose:" +str(self.car_pose))
                if np.any(np.isnan(self.car_pose)) == True:
                    #print("dronepose:" + str(self.drone_pose))        
                    goal.pose.position.x = self.drone_pose[0]
                    goal.pose.position.y = self.drone_pose[1]
                    goal.pose.position.z = self.drone_pose[2] - 0.05
                    if self.drone_pose[2] <= 0.1:
                        os.system('rosservice call /crazyflie3/land')
                        goal.pose.position.z = 0

                else:
                    kpo = 0.5
                    kpi = 0.05
                    # Outer loop Control
                    error[0] = self.car_pose[0] - self.drone_pose[0]
                    error[1] = self.car_pose[1] - self.drone_pose[1]
                    error[2] = self.car_pose[2] - self.drone_pose[2]
                    # Velocity output
                    self.vd[0] = 0.1
                    self.vd[1] = 0
                    self.vd[2] = 0

                    # Inner Loop Control
                    # Velocity estimate
                    v_estimate = [0.0,0.0,0.0]
                    output = [0.0,0.0,0.0]
                    for i in range(3):
                        v_estimate[i] = (self.drone_pose[i] - self.prev_pose[i]) / 0.01
                        output[i] = (self.vd[i] - v_estimate[i])*kpi

                    goal.pose.position.x = 0.05
                    goal.pose.position.y = output[1]
                    goal.pose.position.z = self.car_pose[2] + self.offset_z

                    print('vd: {}'.format(self.vd) + ' va: {}'.format(v_estimate))

                    # Update pose
                    self.prev_pose = self.drone_pose

                # goal.pose.position.x = self.car_pose[0] - displacement[0] + self.offset_x
                # goal.pose.position.y = self.car_pose[1] - displacement[1] + self.offset_y
                # goal.pose.position.z = self.car_pose[2] - displacement[2] + self.offset_z

            self.pubGoal.publish(goal)
            self.counter += 1

