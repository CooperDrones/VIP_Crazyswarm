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

    def getPosePosition(self, vicon_object):
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
        if math.isnan(self.quat_list[0]):
            self.quat_list = [0, 0, 0, 1]
        return self.quat_list
    
    def getPose(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        self.pose1 = self.pose.pose.pose
        return self.pose1

    def commandThrust(self, thrust): # tested and works
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        
        # REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
        for i in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()

        self.msg.linear = Vector3(0, 0, thrust)
        self.msg.angular = Vector3(0, 0, 0)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            self.rate.sleep()
    
    def hoverWithPID(self, z_ref, circle_radius, x_ref, y_ref):
        # REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        for i in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        # Followed paper below section 3.1 for controller
        # https://arxiv.org/pdf/1608.05786.pdf
        # Altitude (z) controller gains and initialization
        self.z_feed_forward = 44705. # Eq. 3.1.8
        self.z_kp = 11000. # Table 3.1.3
        self.z_ki = 3500.
        self.z_kd = 9000
        self.z_error_historical = 0.
        self.thrust_cap_high = 15000 # TODO add caps for all commands
        self.thrust_cap_low = -20000
        self.z_error_before = 0.
        self.z_error_cap = 1.5
        
        # XY controller gains and initialization
        self.x_kp = 10. # Table 3.1.3
        self.x_ki = 2.
        self.y_kp = -10.
        self.y_ki = -2.
        self.x_error_historical = 0.
        self.y_error_historical = 0.
        self.x_before = 0
        self.y_before = 0

        # Yaw rate controller gains
        self.yaw_kp = -20. # Table 3.1.3

        # Set x and y reference values to takeoff position
        origin = self.getPose('crazyflie4')
        x_ref = origin.position.x
        y_ref = origin.position.y
        self.pose_actual = origin
        yaw_ref = 0

        time_step = (1/self.hz)
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
            self.z_error_scaled = (self.z_error * self.z_kp) \
            + (self.z_error_historical * self.z_ki) \
            + (self.z_error_der * self.z_kd) # Eq. 3.1.7

            # publish to thrust command
            self.msg.linear.z = self.z_feed_forward + self.z_error_scaled

            ### XY position controller ###

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
            
            # Find XY error in the drone body frame Eq. 3.1.10
            # Find u (x-velocity) and v (y-velocity) in the body frame
            self.x_error_world = x_ref - self.x_actual
            self.y_error_world = y_ref - self.y_actual

            self.x_e = self.x_error_world * np.cos(self.yaw_angle) \
            + self.y_error_world * np.sin(self.yaw_angle)
            self.u = (self.x_actual - self.x_before) / time_step
            self.x_before = self.x_actual

            self.y_e = -(self.x_error_world * np.sin(self.yaw_angle)) \
            + self.y_error_world * np.cos(self.yaw_angle)
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

            # Plublish commanded actions
            self.msg.linear.x = self.x_error_scaled
            self.msg.linear.y = self.y_error_scaled

            ### Yaw-rate controller Eq. 3.1.13 ###
            self.yaw_error = yaw_ref - self.yaw_angle
            self.yaw_error_scaled = self.yaw_kp * self.yaw_error
            self.msg.angular.z = self.yaw_error_scaled

            ### Useful print statements for debug ###

            # print("The commanded thrust is: {}".format(self.msg.linear.z))
            # print("The z error is {}. Historical error is {}. Derivatice error is {}. Total scaled error is: {}"\
            #     .format(self.z_error, self.z_error_historical, self.z_error_der, self.z_error_scaled)) # HERE
            # print("X command: {}. Y command {}."\
            #     .format(self.x_error_scaled ,self.y_error_scaled))
            # print("The orientation is: {} with type {}".format(self.quat_actual[0], type(self.quat_actual[0])))
            # print('Yaw angle: {}'.format(self.yaw_angle))
            # print('x in body frame: {}'. format(self.x_e))
            # print('y in body frame: {}'. format(self.y_e))

            self.pub.publish(self.msg)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('test')

    try:
        test1 = Tester()

        # # Command just thrust
        # test1.commandThrust(30000)

        # # Command anything for a certain amount of time
        # roll = 0
        # pitch = 0
        # thrust = 10000
        # yaw_rate = 0
        # time_seconds = 0.3
        # test1.commandAction(roll, pitch, thrust, yaw_rate, time_seconds) 

        z_ref = 0.4; circle_radius = 0.1
        test1.hoverWithPID(z_ref, circle_radius, 0.0, 0.0)

    except Exception as e:
        print(e)
