#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
import time
import os
from tf import TransformListener
from std_srvs.srv._Empty import Empty
from geometry_msgs.msg import PoseStamped,Twist
from crazyflie_driver.srv import Takeoff,Land, LandRequest, UpdateParams
from vicon_bridge.srv import viconGrabPose
from control_sys import Control_system

# import logging
# from cflib.crazyflie.log import LogConfig

class Demo():
    def __init__(self, offset_x, offset_y, offset_z, prefix):
        rospy.init_node('demo_cooper', anonymous=True)
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.frame = rospy.get_param("~frame")
        # self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
        self.pubGoalDot = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.listener = TransformListener()
        self.goals = [[0, 0, 0.4, 0, 0]]
        self.goalIndex = 0
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.offset_z = offset_z
        # rospy.wait_for_service(prefix + '/takeoff')
        # self.takeoff_request = rospy.ServiceProxy(prefix + '/takeoff', Takeoff)
        # rospy.wait_for_service(prefix + '/land')
        # self.land_request = rospy.ServiceProxy(prefix + '/takeoff', Land)
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)
        self.record_rate = rospy.Rate(2) # every second
        self.time_delay = rospy.Rate(0.1) # 10 seconds

        self.controllerPos = Control_system(.5,.25)
        self.controllerVel = Control_system(.5)
        # self.lg_var = lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        # self.lg_var.add_variable('stabilizer.roll', 'float')
        # self.lg_var.add_variable('stabilizer.pitch', 'float')
        # self.lg_var.add_variable('stabilizer.yaw', 'float')

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
        
        sp = Twist()
        
        self.counter = 0
        
        prev_T = rospy.Time.now()

        # self.takeoff_request()
        while not rospy.is_shutdown():
            try:
                goal.header.seq += 1
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = self.goals[self.goalIndex][0]
                goal.pose.position.y = self.goals[self.goalIndex][1]
                goal.pose.position.z = self.goals[self.goalIndex][2]

                # This function is returning an invalid quaternion!!!
                # quaternion = np.array([0,0,0,1]) # This allows script to run
                quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])

                goal.pose.orientation.x = quaternion[0]
                goal.pose.orientation.y = quaternion[1]
                goal.pose.orientation.z = quaternion[2]
                goal.pose.orientation.w = quaternion[3]
                error = [0, 0, 0]
                actuator = [0, 0, 0]
                k = 0.5

                if self.counter >= 100:
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


                    
                    if np.any(np.isnan(self.car_pose)) == True:
                        #print("dronepose:" + str(self.drone_pose))        
                        goal.pose.position.x = self.drone_pose[0]
                        goal.pose.position.y = self.drone_pose[1]
                        goal.pose.position.z = self.drone_pose[2] - 0.05
                        if self.drone_pose[2] <= 0.1:
                            os.system('rosservice call /crazyflie3/land')
                            goal.pose.position.z = 0

                    else:
                        # error[0] = self.car_pose[0] - self.drone_pose[0]
                        # error[1] = self.car_pose[1] - self.drone_pose[1]
                        # error[2] = self.car_pose[2] - self.drone_pose[2]
                        # print(error)

                        # goal.pose.position.x = self.car_pose[0] - k * error[0] + self.offset_x
                        # goal.pose.position.y = self.car_pose[1] - k * error[1] + self.offset_y
                        # goal.pose.position.z = self.car_pose[2] + self.offset_z

                        des_pos = np.array(self.car_pose[0:3])
                        des_pos[2] = des_pos[2] + 0.5
                        curr_pos = np.array(self.drone_pose[0:3])

                        des_vel = self.controllerPos.pid(des_pos,curr_pos)
                        # curr_vel = (curr_pos -prev_pos)

                        # for ii in range(0,len(actuator)):
                        #     if actuator[ii] > 1:
                        #         actuator[ii] = 1
                        #     elif actuator[ii] < -1:
                        #         actuator[ii] = -1

                        # goal.pose.position.x = self.car_pose[0] + actuator[0] + self.offset_x
                        # goal.pose.position.y = self.car_pose[1] + actuator[1] + self.offset_y
                        # goal.pose.position.z = self.car_pose[2] + self.offset_z
                        
                        sp.linear.x = des_vel[0]
                        sp.linear.y = des_vel[1]
                        sp.linear.z = des_vel[2]
                        print("Counter = " + str(self.counter))
                        print("x vel: " + str(sp.linear.x))
                        print("y vel: " + str(sp.linear.y))
                        print("z vel: " + str(sp.linear.z))
                        

                    # goal.pose.position.x = self.car_pose[0] - displacement[0] + self.offset_x
                    # goal.pose.position.y = self.car_pose[1] - displacement[1] + self.offset_y
                    # goal.pose.position.z = self.car_pose[2] - displacement[2] + self.offset_z

                #self.pubGoal.publish(goal)
                self.pubGoalDot.publish(sp)
                self.counter += 1
            
            except Exception as e:
                print(e)

