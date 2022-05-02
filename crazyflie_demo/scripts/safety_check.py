#!/usr/bin/env python

import imp
import numpy as np
import pickle
from datetime import datetime
from pytz import timezone
import os
import sys
import time
import psutil

# Import crazyflie model modules
from a_cf_controller_phys import AltitudeControllerPhys, XYControllerPhys, YawControllerPhys, XYControllerTrajPhys
from data_plotter import DataPlotter
import crazyflie_param as P
from tf import TransformListener

# Import ros specifc modules
import rospy
from geometry_msgs.msg import Twist, Vector3, TransformStamped, PoseStamped # twist used in cmd_vel
from vicon_bridge.srv import viconGrabPose
from std_msgs.msg import Int8
from crazyflie_driver.srv import Takeoff,Land, LandRequest


class safety_check:
    def __init__(self, safety_bound_x=None, safety_bound_y=None, safety_bound_z=None, buffer_bound_x=None, buffer_bound_y=None,buffer_bound_z=None):
        self.safety_bound_x = safety_bound_x
        self.safety_bound_y = safety_bound_y
        self.safety_bound_z = safety_bound_z
        self.buffer_bound_x = buffer_bound_x
        self.buffer_bound_y = buffer_bound_y
        self.buffer_bound_z = buffer_bound_z

    def bound_check(self, x, y, z):
        if (np.abs(x) >= self.safety_bound_x and np.abs(x) <= self.buffer_bound_x):
            print("Position x is in buffer zone")
        elif (np.abs(x) >= self.buffer_bound_x):
            print("Position x is in danger zone!")
            os.system('rosservice call /crazyflie3/land')
            time.sleep(3)
        if (np.abs(y) >= self.safety_bound_y and np.abs(y) <= self.buffer_bound_y):
            print("Position y is in buffer zone")
        elif (np.abs(y) >= self.buffer_bound_y):
            print("Position y is in danger zone!")
            os.system('rosservice call /crazyflie3/land')
            time.sleep(3)
        if (np.abs(z) >= self.safety_bound_z and np.abs(z) <= self.buffer_bound_z):
            print("Position z is in buffer zone")
        elif (np.abs(z) >=  self.buffer_bound_z):
            print("Position z is in danger zone!")
            os.system('rosservice call /crazyflie3/land')
            time.sleep(3)
        return 0

    # def __init__(self, cf_name, is_main=False):
    #     rospy.init_node('demo_cooper', anonymous=True)
    #     self.cf_name = cf_name
    #     self.worldFrame = rospy.get_param("~worldFrame", "/world")
    #     self.frame = rospy.get_param("~frame")
    #     # self.pubGoal = rospy.Publisher('goal', PoseStamped, queue_size=1)
    #     self.listener = TransformListener()
    #     self.listener = TransformListener()
    #     self.goals = [[0, 0, 0.4, 0, 0]]
    #     # rospy.wait_for_service(prefix + '/takeoff')        
    #     self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)
    #     self.record_rate = rospy.Rate(2) # every second
    #     self.time_delay = rospy.Rate(0.1) # 10 seconds

    # def getPose(self, vicon_object):
    #     self.pose = self.pose_getter(vicon_object, vicon_object, 1)
    #     self.position = self.pose.pose.pose.position
    #     self.position_list = [self.position.x, self.position.y, self.position.z]
    #     print(self.position_list)
    #     return self.position_list

    # def run(self):
    #     self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0))
    #     goal = PoseStamped()
    #     goal.header.seq = 0
    #     goal.header.frame_id = self.worldFrame
    #     self.counter = 0
    #     # self.takeoff_request()
    #     while not rospy.is_shutdown():
    #         goal.header.seq += 1
    #         goal.header.stamp = rospy.Time.now()
    #         goal.pose.position.x = self.goals[self.goalIndex][0]
    #         goal.pose.position.y = self.goals[self.goalIndex][1]
    #         goal.pose.position.z = self.goals[self.goalIndex][2]
    #         quaternion = tf.transformations.quaternion_from_euler(0, 0, self.goals[self.goalIndex][3])
    #         goal.pose.orientation.x = quaternion[0]
    #         goal.pose.orientation.y = quaternion[1]
    #         goal.pose.orientation.z = quaternion[2]
    #         goal.pose.orientation.w = quaternion[3]

    #         self.pubGoal.publish(goal)
    #         self.counter += 1

    # TODO: need to get the actual dimension of the boundaries
# def safe_bound():
#     x = 5
#     y = 3
#     z = 2
#     return x,y,z

# def buffer_bound():
#     x = 5.2
#     y = 3.2
#     z = 2.2
#     return x,y,z

# def pose_check(pose):
#     # In the safe zone
#     if (pose <= safe_bound):
#         return 0
#     # In the buffer zone
#     if (pose > safe_bound and pose <= buffer_bound):
#         return 1
#     # In the danger zone
#     if (pose > buffer_bound):
#         return 2

    
    #     self.msg = Twist()
    #     self.hz = 30.0
    #     self.t_phys = 1/self.hz # TODO make P.t_phys import
    #     self.rate = rospy.Rate(self.hz)
        
    #     # Requried for landing script
    #     self.x_fin = 0.0 
    #     self.y_fin = 0.0
    #     self.z_fin = 0.0
    #     self.yaw_fin = 0.0

    #     # Required based on syntax of launch script
    #     if is_main:
    #         self.pub = rospy.Publisher(self.cf_name + "/sf_cmd_vel", Twist, queue_size=10)
    #     else:
    #         self.pub = rospy.Publisher("sf_cmd_vel", Twist, queue_size=10)

    #     # Required for vicon listener
    #     self.pose = TransformStamped()
    #     self.pose.transform.rotation.w = 1.0

    #     # Requied for publishPostion function
    #     self.pubPos = rospy.Publisher('cf_pos', PoseStamped, queue_size=1)

    # def callback(self, pose):
    #     self.pose = pose
    
    # def listener(self):
    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    #     pose = self.pose
    #     rospy.spin()
    
    # # def callbackRef(self, pose):
    # #     self.pos_ref = pose

    # # def listener_ref(self, ref_object):
    # #     rospy.Subscriber("/vicon/" + ref_object + "/" + ref_object, TransformStamped, self.callbackRef)
    # #     pose_ref

    # def dummyForLoop(self):
    #     """
    #     REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
    #     """
    #     self.msg.linear = Vector3(0, 0, 0)
    #     self.msg.angular = Vector3(0, 0, 0)
    #     for _ in range(100):
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()
    
    # # def publishPosition(self):
    # #     # Start the vicon listener
    # #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    # #     pose = self.pose


    # def safetyCheck(self, x_c, y_c, z_c, yaw_c, goal_r, is_break=True, \
    #     is_synchronized=False, global_sync_time=0.0, time_delay=None, var=None):
    #     """
    #     Hovers the drone to an accurate global setpoint
    #     Drone will stay at setpoint until other function is called
    #     Stiff refers to optimization for global positional accuracy

    #     Parameters
    #     ----------
    #     x_c, y_c, z_c, yaw_c = reference setpoints
    #     goal_r = bounding radius for when drone is "close enough" to commanded setpoint
    #     is_break = will the loop break upon finding the setpoint or hover
    #         indefinitely
    #     is_synchornized = incorporate a universal time delay such that drones
    #         all start trajectory tracking at the same time
    #     global_sync_time = length of time delay for synchronization efforts
    #     """
    #     print(self.cf_name + ' started hover controller')

    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    #     pose = self.pose

    #     # Initialize required hover controllers
    #     altitude_ctrl_phys = AltitudeControllerPhys()
    #     xy_ctrl_phys = XYControllerPhys()
    #     yaw_ctrl_phys = YawControllerPhys()

    #     # # for is_synchronized overload
    #     # tz = timezone('EST')
    #     # now = datetime.now(tz)
    #     # secs = str(now.second) + '.' + str(now.microsecond)
    #     # local_now = now.hour * 3600 + now.minute * 60 + float(secs)
    #     # local_in_loop = local_now
        
    #     # for time_delay overload
    #     local2 = 0.0
        
    #     while not rospy.is_shutdown():
    #         pose_prev = pose
    #         pose = self.pose
    #         print(pose.transform.translation)
    #         quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
    #         x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
    #         if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
    #             pose = pose_prev

    #         # Obtain yaw angle from quaternion
    #         R = Rotation.from_quat(quat)
    #         x_global = R.apply([1, 0, 0]) # project to world x-axis
    #         yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            

    # def trajTrackingStandingWave(self, traj, z_c, y_c=0.0):
    #     """
    #     Runs a 2D trajectory tracking algorithm in the XY plane

    #     Parameters
    #     ----------
    #     traj = trajectory that increments at each loop iteration
    #     z_c  = commanded height value
    #     y_c  = commanded y-offset value if traj only operates in x direction
    #     """
    #     print(self.cf_name + ' started trajectory tracking!')

    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    #     pose = self.pose

    #     # Initialize required controllers
    #     altitude_ctrl_phys = AltitudeControllerPhys()
    #     xy_traj_ctrl_phys = XYControllerTrajPhys()
    #     yaw_ctrl_phys = YawControllerPhys()
        
    #     y_c = y_c; v_c = 0.0; vd_c = 0.0 # keep y values equal to zero for now 
    #     yaw_c = 0.0

    #     # Will finish at end of trajectory matrix, 1 entry per loop interation
    #     for i in range(traj.shape[0] - 1):
    #         pose_prev = pose
    #         pose = self.pose
    #         quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
    #         x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
    #         if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
    #             pose = pose_prev
        
    #         # Obtain yaw angle from quaternion
    #         R = Rotation.from_quat(quat)
    #         x_global = R.apply([1, 0, 0]) # project to world x-axis
    #         yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

    #         # TODO: make flexible with y values
    #         r_t      = np.array([traj[i, 0], y_c]) # traj pos values
    #         r_t_vect = np.array([traj[i+1, 0], y_c]) - r_t # vector from current pos to next pos in traj
    #         rd_t     = np.array([traj[i, 1], v_c]) # traj vel values
    #         rdd_t    = np.array([traj[i, 2], vd_c])
    #         r        = np.array([x, y]) # actual drone pos

    #         self.msg.linear.z = altitude_ctrl_phys.update(z_c, z)
    #         self.msg.linear.x, self.msg.linear.y = xy_traj_ctrl_phys.update(r_t, rd_t, r_t_vect, r, yaw_c, rdd_t, True)
    #         self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)

    #         # print('theta (commands +x) is: ', self.msg.linear.x)
    #         # print('phi   (commands -y) is: ', self.msg.linear.y)

    #         # self.msg.linear.x = 0.0; self.msg.linear.y = 0.0 # set to tune trim values

    #         self.pub.publish(self.msg)
    #         self.rate.sleep()

    #     print(self.cf_name + ' completed trajectory tracking!')

    #     # Save out data through pickle
    #     xy_traj_ctrl_phys.pickleData()




    # def trajTracking(self, traj, z_c):
    #     """
    #     Runs a 2D-3D trajectory tracking algorithm

    #     Parameters
    #     ----------
    #     traj = trajectory that increments at each loop iteration in form
    #     [ x, y, z, xd, yd, zd, xdd, ydd, zdd ]
    #     ref_object = where position readings are referenced, default is the world origin
    #     """
    #     print(self.cf_name + ' started trajectory tracking!')

    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    #     pose = self.pose

    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)

    #     # Initialize required controllers
    #     altitude_ctrl_phys = AltitudeControllerPhys()
    #     xy_traj_ctrl_phys = XYControllerTrajPhys()
    #     yaw_ctrl_phys = YawControllerPhys()

    #     yaw_c = 0.0

    #     # Will finish at end of trajectory matrix, 1 entry per loop interation
    #     for i in range(traj.shape[0] - 1):
    #         pose_prev = pose
    #         pose = self.pose
    #         quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
    #         x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
    #         if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
    #             pose = pose_prev
        
    #         # Obtain yaw angle from quaternion
    #         R = Rotation.from_quat(quat)
    #         x_global = R.apply([1, 0, 0]) # project to world x-axis
    #         yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

    #         #cf1.hoverStiff(0.0, 0.0, 0.4, 0.0, 0.01) # False means don't break after reaching set point
    #         r_t      = np.array([traj[i, 0], traj[i, 3]]) # traj pos values
    #         r_t_vect = np.array([traj[i+1, 0], traj[i+1, 3]]) - r_t # vector from current pos to next pos in traj
    #         rd_t     = np.array([traj[i, 1], traj[i, 4]]) # traj vel values
    #         rdd_t    = np.array([traj[i, 2], traj[i, 5]])
    #         r        = np.array([x, y]) # actual drone pos

    #         self.msg.linear.z = altitude_ctrl_phys.update(z_c, z)
    #         self.msg.linear.x, self.msg.linear.y = xy_traj_ctrl_phys.update(r_t, rd_t, r_t_vect, r, yaw_c, rdd_t, is_pickling=True)
    #         self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)

    #         # print('theta (commands +x) is: ', self.msg.linear.x)
    #         # print('phi   (commands -y) is: ', self.msg.linear.y)

    #         self.pub.publish(self.msg)
    #         self.rate.sleep()

    #     print(self.cf_name + ' completed trajectory tracking!')

    #     # Save out data through pickle
    #     xy_traj_ctrl_phys.pickleData()

    # def land(self):
    #     """
    #     Take last known x and y coordinates from hover script
    #     and slowly land the drone while maintaining xy control
    #     """
    #     print(self.cf_name + ' started landing!')

    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    #     pose = self.pose

    #     # Initialize required hover controllers
    #     xy_ctrl_phys = XYControllerPhys(cap=5.0)
    #     yaw_ctrl_phys = YawControllerPhys()

    #     x_c = self.x_fin; y_c = self.y_fin; yaw_c = self.yaw_fin
    #     self.msg.linear.z = 40000.0
        
    #     while not rospy.is_shutdown():
    #         pose_prev = pose
    #         pose = self.pose
    #         quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
    #         x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
    #         if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
    #             pose = pose_prev
            
    #         # Obtain yaw angle from quaternion
    #         R = Rotation.from_quat(quat)
    #         x_global = R.apply([1, 0, 0]) # project to world x-axis
    #         yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

    #         self.msg.linear.x, self.msg.linear.y = xy_ctrl_phys.update(x_c, x, y_c, y, yaw)
    #         self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)
                
    #         if self.msg.linear.z > 35000.0:
    #             self.msg.linear.z -= 150.0
    #         else:    
    #             print(self.cf_name + ' completed landing!')
    #             break

    #         self.pub.publish(self.msg)
    #         self.rate.sleep()


