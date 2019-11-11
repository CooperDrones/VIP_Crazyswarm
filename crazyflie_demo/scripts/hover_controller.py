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
        self.position = self.pose.pose.pose.position
        self.position_list = [self.position.x, self.position.y, self.position.z]
        if math.isnan(self.position_list[0]):
            self.posiiton_list = [0, 0, 0]
        return self.position_list
    
    def getPoseQuaternion(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        self.quat = self.pose.pose.pose.orientation
        self.quat_list = [self.quat.x, self.quat.y, self.quat.z, self.quat.w]
        if math.isnan(self.position_list[0]):
            self.quat_list = [0, 0, 0, 1]
        return self.quat_list

    def commandThrust(self, thrust): # tested and works
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        for i in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()
        self.msg.linear = Vector3(0, 0, thrust)
        self.msg.angular = Vector3(0, 0, 0)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            self.rate.sleep()
    
    def hoverWithBangBang(self, z_ref, circle_radius, x_ref, y_ref):
        # REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        for i in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        # Altitude (z) controller gains
        self.z_feed_forward = 45000.
        self.z_kp = 11000.
        self.z_ki = 3500.
        self.z_kd = 9000
        self.z_error_historical = 0.
        self.z_feed_forward = 39000.
        self.z_error_cap = 1.5 # prevent wind up
        self.z_error_before = 0.
        
        # BANG BANG gains
        self.xy_kp = 10.
        
        # XY controller gains
        self.x_kp = 10.
        self.x_ki = 2.
        self.y_kp = 10.
        self.y_ki = 2.

        self.x_error_historical = 0.
        self.y_error_historical = 0.

        # Yaw rate controller gains
        self.yaw_kp = -30.

        # self.x_origin = 0.
        # self.y_origin = 0.

        time_step = (1/self.hz)
        while not rospy.is_shutdown():
            self.position_actual = self.getPose('crazyflie3')
            # print("The position is: {}".format(self.position_actual))

            # Trim based on observation
            self.msg.linear.x = 0
            self.msg.linear.y = 0

            # Altitude controller
            self.z_actual = self.position_actual[2]

            # Proportional controller
            self.z_error = z_ref - self.z_actual
            
            # Integral controller with bound
            if self.z_error_historical <= self.z_error_cap:
                self.z_error_historical += (self.z_error * time_step)
            
            # Derivative controller
            # print(0.03333)
            # print(self.z_error - self.z_error_before)

            self.z_error_der = (self.z_error - self.z_error_before) / (0.03333) # HERE
            self.z_error_before = self.z_error # HERE

            # Add errors together and multiply by gains
            self.z_error_scaled = (self.z_error * self.z_kp) + (self.z_error_historical * self.z_ki) \
                + (self.z_error_der * self.z_kd) # HERE

            # publish to thrust command
            self.msg.linear.z = self.z_feed_forward + self.z_error_scaled
            
            # Lateral bang bang controller - provides restoring pitch and yaw if drone excapes circle on xy plane
            # if self.counter >= 15:
            self.x_actual = self.position_actual[0]
            self.y_actual = self.position_actual[1]

            # BANG BANG Controller
            # self.offset = np.sqrt((self.x_actual - self.x_origin)**2 + (self.y_actual - self.y_origin)**2)
            # self.theta = np.arctan2((self.y_actual - self.y_origin), (self.x_actual - self.x_origin))

            # self.msg.linear.x = -1 * self.xy_kp * self.offset * np.cos(self.theta)
            # self.msg.linear.y = self.xy_kp * self.offset * np.sin(self.theta)

            self.x_error = x_ref - self.x_actual
            self.y_error = y_ref - self.y_actual

            self.x_error_historical += (self.z_error * time_step)
            self.y_error_historical += (self.z_error * time_step)

            self.x_error_scaled = (self.x_error * self.x_kp) + (self.x_error_historical * self.x_ki)
            self.y_error_scaled = (self.y_error * self.y_kp) + (self.y_error_historical * self.y_ki)

            self.msg.linear.x = self.x_error_scaled
            self.msg.linear.y = -self.y_error_scaled

            # Yaw-rate controller
            self.quat_actual = self.getPoseQuaternion('crazyflie3')
            R = Rotation.from_quat(self.quat_actual)
            self.global_x = R.apply([1, 0, 0]) # project to world x-axis
            self.yaw_angle = (-1) * np.arctan2(np.cross([1, 0, 0], self.global_x)[2], np.dot(self.global_x, [1, 0, 0]))
            self.msg.angular.z = self.yaw_kp * self.yaw_angle

            # print("The commanded thrust is: {}".format(self.msg.linear.z))
            # print("The z error is {}. Historical error is {}. Derivatice error is {}. Total scaled error is: {}"\
            #     .format(self.z_error, self.z_error_historical, self.z_error_der, self.z_error_scaled)) # HERE
            # print("The X error is {}. Y error is {}."\
                # .format(self.x_error_scaled ,self.y_error_scaled))
            # print("The orientation is: {} with type {}".format(self.quat_actual[0], type(self.quat_actual[0])))

            self.pub.publish(self.msg)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('test')


    try:
        test1 = Tester()
        # test1.takeOff(0.4) # Using hover msg which uses zdistance

        # Command just thrust
        # while not rospy.is_shutdown():
        #     test1.commandThrust(30000)

        # Command anything for a certain amount of time
        # roll = 0
        # pitch = 0
        # thrust = 10000
        # yaw_rate = 0
        # time_seconds = 0.3
        # test1.commandAction(roll, pitch, thrust, yaw_rate, time_seconds) 

        # z_ref = 0.5 # command height in meters
        # test1.hoverWithFeedback(z_ref)

        z_ref = 0.5; circle_radius = 0.1
        test1.hoverWithBangBang(z_ref, circle_radius, 0.0, 0.0)

        # test1.listenerTest()

    except Exception as e:
        print(e)
