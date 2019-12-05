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
        self.hz = 30.0
        self.time_step = (1/self.hz)
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size=0)
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)

        # Followed this paper, section 3.1, for PID controller
        # https://arxiv.org/pdf/1608.05786.pdf
        # Altitude (z) controller gains and initialization
        self.z_feed_forward = 41000. # Eq. 3.1.8 - a bit less since we do not use UWB module
        self.z_kp = 11000. # Table 3.1.3
        self.z_ki = 3500.
        self.z_kd = 9000.
        self.thrust_cap_high = 15000 # TODO add caps for all commands
        self.thrust_cap_low = -20000
        self.ze_cap = 1.5

        # xy controller gains and initialization
        self.x_kp = 10. # Table 3.1.3
        self.x_ki = 2.
        self.y_kp = -10.
        self.y_ki = -2.
        self.x_cap = 20.
        self.y_cap = 20.

        # Yaw rate controller gains
        self.yaw_kp = -20. # Table 3.1.3

        # Velocity controller gains
        self.u_kp = 5
        self.v_kp = -5

    def getPose(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        self.pose1 = self.pose.pose.pose
        return self.pose1

    def dummyForLoop(self):
        # REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        for _ in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()

    def hover(self, xr, yr, zr, goal_r):
        print('Start hover controller')
        # Initialize function specfic values
        ze_hist = 0.
        ze_prev = 0.
        x_b_prev = 0.
        xe_b_hist = 0.
        y_b_prev = 0.
        ye_b_hist = 0.
        origin = self.getPose('crazyflie4')
        pose = origin
        yawr = 0.
        
        while not rospy.is_shutdown():
            # Get current drone pose
            pose_prev = pose
            pose = self.getPose('crazyflie4')
            if math.isnan(pose.orientation.x): # If nan is thrown, set to last known position
                pose = pose_prev

            ### Altitude controller ###
            z = pose.position.z # Get true z value
            ze = zr - z # Get error
            if ze_hist <= self.ze_cap: # prevent wind-up
                ze_hist += (ze * self.time_step) # Find integral component
            ze_der = (ze - ze_prev) / self.time_step # Find derivative component
            ze_prev = ze
            ze_tot = (ze * self.z_kp) + (ze_hist * self.z_ki) \
                + (ze_der * self.z_kd) # Eq. 3.1.7 Sum PID errors and multiply by gains
            self.msg.linear.z = self.z_feed_forward + ze_tot

            ### xy position controller ###
            x = pose.position.x; y = pose.position.y # Get true x and y values
            xe = xr - x; ye = yr - y # Get position error

            # Obtain yaw angle from quaternion
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))
            
            x_b = x * np.cos(yaw) + y * np.sin(yaw) # Get x in body frame
            u = (x_b - x_b_prev) / self.time_step # u is x-vel in body frame
            x_b_prev = x_b # Reset previous val
            y_b = -(x * np.sin(yaw)) + y * np.cos(yaw) # Get y in body frame
            v = (y_b - y_b_prev) / self.time_step # v is y-vel in body frame
            y_b_prev = y_b # Reset previous val
            xe_b = xe * np.cos(yaw) + ye * np.sin(yaw) # Get errors in body frame
            ye_b = -(xe * np.sin(yaw)) + ye * np.cos(yaw)
            xe_b_hist += ((xe_b - u) * self.time_step) # Accumulate and store histroical error
            ye_b_hist += ((ye_b - v) * self.time_step)
            xe_b_tot = ((xe_b - u) * self.x_kp) + (xe_b_hist * self.x_ki) # Eq. 3.1.11 and Eq. 3.1.12
            ye_b_tot = ((ye_b - v) * self.y_kp) + (ye_b_hist * self.y_ki)

            # Cap roll (y) and pitch (x) to prevent unstable maneuvers
            if xe_b_tot >= self.x_cap:
                xe_b_tot = self.x_cap
            elif xe_b_tot <= -self.x_cap:
                xe_b_tot = -self.x_cap
            elif ye_b_tot >= self.y_cap:
                ye_b_tot = self.y_cap            
            elif ye_b_tot <= -self.y_cap:
                ye_b_tot = -self.y_cap

            self.msg.linear.x = xe_b_tot
            self.msg.linear.y = ye_b_tot

            ### yaw-rate controller Eq. 3.1.13 ###
            yawe = yawr - yaw
            yawe_tot = self.yaw_kp * yawe
            self.msg.angular.z = yawe_tot

            ### Goal behavior ###
            if (x > (xr - goal_r) and x < (xr + goal_r)) and \
                (y > (yr - goal_r) and y < (yr + goal_r)) and \
                (z > (zr - goal_r) and z < (zr + goal_r)):
                print('Found the hover setpoint!')
                break

            self.pub.publish(self.msg)
            self.rate.sleep()

def main():
    rospy.init_node('test')

    try:
        drone1 = Tester()

        drone1.dummyForLoop()

        # Hover at origin
        xr = 0.0; yr = 0.0; zr = 0.4 # [m]
        goal_r = 0.05
        drone1.hover(xr, yr, zr, goal_r)

        # land the drone
        zr = 0.15
        drone1.hover(xr, yr, zr, goal_r)

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()