#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3,TransformStamped # twist used in cmd_vel
from crazyflie_driver.msg import Hover # used in cmd_hover commands vel, yaw rate, and hover height
from crazyflie_driver.srv import Takeoff
from std_msgs.msg import Duration
from vicon_bridge.srv import viconGrabPose
import numpy as np
from scipy.spatial.transform import Rotation
import math
import scipy.interpolate as si
import matplotlib.pyplot as plt

class Tester:
    def __init__(self):
        self.msg = Twist()
        self.hz = 30.0 # if not set to 100, will not broadcast
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size=0)
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)
    
    def getPose(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        self.pose1 = self.pose.pose.pose
        return self.pose1
        
    def hover(self, waypoints, circle_radius):
        # REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        for _ in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        # Followed this paper, section 3.1, for PID controller
        # https://arxiv.org/pdf/1608.05786.pdf
        # Altitude (z) controller gains and initialization
        self.z_feed_forward = 44000. # Eq. 3.1.8 - a bit less since we do not use UWB module
        self.z_kp = 11000. # Table 3.1.3
        self.z_ki = 3500.
        self.z_kd = 9000.
        self.z_error_historical = 0.
        self.thrust_cap_high = 15000 # TODO add caps for all commands
        self.thrust_cap_low = -20000
        self.z_error_before = 0.
        self.z_error_cap = 1.5
        
        # xy controller gains and initialization
        self.x_kp = 10. # Table 3.1.3
        self.x_ki = 2.
        self.y_kp = -10.
        self.y_ki = -2.
        self.x_error_historical = 0.
        self.y_error_historical = 0.
        self.x_before = 0.
        self.y_before = 0.
        self.x_cap = 30.
        self.y_cap = 30.

        # Yaw rate controller gains
        self.yaw_kp = -20. # Table 3.1.3

        # Set initial reference values
        # x_ref = waypoints[0,0]; y_ref = waypoints[0,1]; z_ref = waypoints[0,2]
        origin = self.getPose('crazyflie4')
        self.pose_actual = origin
        
        # Hold yaw constant throughout
        yaw_ref = 0
        time_step = (1/self.hz)
        x_ref = 0.0; y_ref = 0.0

        while not rospy.is_shutdown():
            # Get current drone pose
            self.pose_before = self.pose_actual
            self.pose_actual = self.getPose('crazyflie4')
            if math.isnan(self.pose_actual.orientation.x): # If nan is thrown, set to last known position
                self.pose_actual = self.pose_before

            ### Altitude controller ###

            # Get true z value
            self.z_actual = self.pose_actual.position.z

            # Get error
            self.z_error = z_ref - self.z_actual
            
            # Find integral component
            if self.z_error_historical <= self.z_error_cap:
                self.z_error_historical += (self.z_error * time_step)
            
            # Find derivative component
            self.z_error_der = (self.z_error - self.z_error_before) / time_step
            self.z_error_before = self.z_error

            # Sum PID errors and multiply by gains
            self.z_error_scaled = (self.z_error * self.z_kp) + (self.z_error_historical * self.z_ki) \
                + (self.z_error_der * self.z_kd) # Eq. 3.1.7

            # publish to thrust command
            self.msg.linear.z = self.z_feed_forward + self.z_error_scaled

            ### xy position controller ###

            # get true x and y values
            self.x_actual = self.pose_actual.position.x
            self.y_actual = self.pose_actual.position.y

            # Obtain yaw angle from quaternion
            self.quat_actual = [self.pose_actual.orientation.x, self.pose_actual.orientation.y, \
                self.pose_actual.orientation.z, self.pose_actual.orientation.w]
            R = Rotation.from_quat(self.quat_actual)
            self.global_x = R.apply([1, 0, 0]) # project to world x-axis
            self.yaw_angle = np.arctan2(np.cross([1, 0, 0], self.global_x)[2], \
                np.dot(self.global_x, [1, 0, 0]))
            
            # obtain position error
            self.x_error_world = x_ref - self.x_actual
            self.y_error_world = y_ref - self.y_actual

            # x-position controller
            self.x_e = self.x_error_world * np.cos(self.yaw_angle) + self.y_error_world * np.sin(self.yaw_angle)
            self.u = (self.x_actual - self.x_before) / time_step
            self.x_before = self.x_actual

            # y-position controller
            self.y_e = -(self.x_error_world * np.sin(self.yaw_angle)) + self.y_error_world * np.cos(self.yaw_angle)
            self.v = (self.y_actual - self.y_before) / time_step
            self.y_before = self.y_actual

            # Eq. 3.1.11 and Eq. 3.1.12
            self.x_diff = self.x_e - self.u
            self.y_diff = self.y_e - self.v

            # Find integral component - store historical error
            self.x_error_historical += (self.x_diff * time_step)
            self.y_error_historical += (self.y_diff * time_step)

            # Sum PI errors and multiply by gains
            self.x_error_scaled = (self.x_diff * self.x_kp) \
                + (self.x_error_historical * self.x_ki)
            self.y_error_scaled = (self.y_diff * self.y_kp) \
                + (self.y_error_historical * self.y_ki)

            # Cap errors to prevent unstable maneuvers
            if self.y_error_scaled >= self.y_cap:
                self.y_error_scaled = self.y_cap
            
            elif self.y_error_scaled <= -self.y_cap:
                self.y_error_scaled = -self.y_cap

            # Plublish commanded actions
            self.msg.linear.x = self.x_error_scaled
            self.msg.linear.y = self.y_error_scaled

            ### Yaw-rate controller Eq. 3.1.13 ###
            self.yaw_error = yaw_ref - self.yaw_angle
            self.yaw_error_scaled = self.yaw_kp * self.yaw_error
            self.msg.angular.z = self.yaw_error_scaled

            ### Useful print statements for debug ###

            # Kills hover once at stable position last statement ensures drone will stay at last point
            if (self.x_actual > (x_ref - circle_radius) and self.x_actual < (x_ref + circle_radius)) and \
                (self.y_actual > (y_ref - circle_radius) and self.y_actual < (y_ref + circle_radius)) and \
                (self.z_actual > (z_ref - circle_radius) and self.z_actual < (z_ref + circle_radius)):
                break
            
            self.pub.publish(self.msg)
            self.rate.sleep()

    # def pathTracker(self, u_ref, distance):
    #     x_ref = self.x_actual + distance

    #     while not rospy.is_shutdown():
    #         # Get current drone pose
    #         self.pose_before = self.pose_actual
    #         self.pose_actual = self.getPose('crazyflie4')
    #         if math.isnan(self.pose_actual.orientation.x): # If nan is thrown, set to last known position
    #             self.pose_actual = self.pose_before

    #         # Set reference reference values
    #         y_ref = 0.0

    #         ### Altitude controller ###

    #         # Get true z value
    #         self.z_actual = self.pose_actual.position.z

    #         # Get error
    #         self.z_error = z_ref - self.z_actual
            
    #         # Find integral component
    #         if self.z_error_historical <= self.z_error_cap:
    #             self.z_error_historical += (self.z_error * time_step)
            
    #         # Find derivative component
    #         self.z_error_der = (self.z_error - self.z_error_before) / time_step
    #         self.z_error_before = self.z_error

    #         # Sum PID errors and multiply by gains
    #         self.z_error_scaled = (self.z_error * self.z_kp) + (self.z_error_historical * self.z_ki) \
    #             + (self.z_error_der * self.z_kd) # Eq. 3.1.7

    #         # publish to thrust command
    #         self.msg.linear.z = self.z_feed_forward + self.z_error_scaled

    #         ### xy position controller ###

    #         # get true x and y values
    #         self.x_actual = self.pose_actual.position.x
    #         self.y_actual = self.pose_actual.position.y

    #         # Obtain yaw angle from quaternion
    #         self.quat_actual = [self.pose_actual.orientation.x, self.pose_actual.orientation.y, \
    #             self.pose_actual.orientation.z, self.pose_actual.orientation.w]
    #         R = Rotation.from_quat(self.quat_actual)
    #         self.global_x = R.apply([1, 0, 0]) # project to world x-axis
    #         self.yaw_angle = np.arctan2(np.cross([1, 0, 0], self.global_x)[2], \
    #             np.dot(self.global_x, [1, 0, 0]))
            
    #         # x-position controller
    #         self.x_error_world = x_ref - self.x_actual
    #         self.x_e = self.x_error_world * np.cos(self.yaw_angle) + self.y_error_world * np.sin(self.yaw_angle)
    #         self.u = (self.x_actual - self.x_before) / time_step
    #         self.x_before = self.x_actual

    #         # y-position controller
    #         self.y_error_world = y_ref - self.y_actual
    #         self.y_e = -(self.x_error_world * np.sin(self.yaw_angle)) + self.y_error_world * np.cos(self.yaw_angle)
    #         self.v = (self.y_actual - self.y_before) / time_step
    #         self.y_before = self.y_actual

    #         # Eq. 3.1.11 and Eq. 3.1.12
    #         self.x_diff = self.x_e - self.u
    #         self.y_diff = self.y_e - self.v

    #         # Find integral component - store historical error
    #         self.x_error_historical += (self.x_diff * time_step)
    #         self.y_error_historical += (self.y_diff * time_step)

    #         # Sum PI errors and multiply by gains
    #         self.x_error_scaled = (self.x_diff * self.x_kp) \
    #             + (self.x_error_historical * self.x_ki)
    #         self.y_error_scaled = (self.y_diff * self.y_kp) \
    #             + (self.y_error_historical * self.y_ki)

    #         # Cap errors to prevent unstable maneuvek
    #         if self.y_error_scaled >= self.y_cap:
    #             self.y_error_scaled = self.y_cap
            
    #         elif self.y_error_scaled <= -self.y_cap:
    #             self.y_error_scaled = -self.y_cap

    #         # Plublish commanded actions
    #         self.msg.linear.x = self.x_error_scaled
    #         self.msg.linear.y = self.y_error_scaled

    #         ### Yaw-rate controller Eq. 3.1.13 ###
    #         self.yaw_error = yaw_ref - self.yaw_angle
    #         self.yaw_error_scaled = self.yaw_kp * self.yaw_error
    #         self.msg.angular.z = self.yaw_error_scaled

    #         ### Useful print statements for debug ###

    #         # Kills hover once at stable position last statement ensures drone will stay at last point
    #         if (self.x_actual > (x_ref - circle_radius) and self.x_actual < (x_ref + circle_radius)) and \
    #             (self.y_actual > (y_ref - circle_radius) and self.y_actual < (y_ref + circle_radius)) and \
    #             (self.z_actual > (z_ref - circle_radius) and self.z_actual < (z_ref + circle_radius)):
    #             break
            
    #         self.pub.publish(self.msg)
    #         self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('test')

    try:
        drone1 = Tester()

        circle_radius = 0.1 # m
        z_ref = 0.3 # m
        drone1.hover(z_ref, circle_radius)

        # u_ref = 0.5 # m/s
        # drone1.pathTracker(u_ref, distance)

    except Exception as e:
        print(e)