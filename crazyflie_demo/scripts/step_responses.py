#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3,TransformStamped # twist used in cmd_vel
from crazyflie_driver.msg import Hover # used in cmd_hover commands vel, yaw rate, and hover height
from crazyflie_driver.srv import Takeoff
from std_msgs.msg import Duration
from vicon_bridge.srv import viconGrabPose
import numpy as np
from scipy.spatial.transform import Rotation

class Tester:
    def __init__(self):

        # worldFrame = rospy.get_param("~worldFrame", "/world")      
        self.sub = rospy.Subscriber("vicon/crazyflie2/crazyflie2", TransformStamped, self.callback)
        self.msg = Hover()
        self.twist_msg = Twist()
        self.duration_msg = Duration()
        # self.duration_msg.data.secs = 10f
        self.hz = 100
        self.rate = rospy.Rate(self.hz)

        # rospy.wait_for_service('/crazyflie/takeoff')
        # self.takeoff_command = rospy.ServiceProxy('/crazyflie/takeoff', Takeoff)

        self.pub = rospy.Publisher("crazyflie2/cmd_hover", Hover, queue_size=0)
        self.pub_twist = rospy.Publisher("crazyflie2/cmd_vel", Twist, queue_size=0)

        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)

    def callback(self, data):
        self.current_state = data

    def getPose(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        return self.pose.pose.pose.position
    
    def getPoseQuaternion(self, vicon_object):
        self.pose = self.pose_getter(vicon_object, vicon_object, 1)
        return self.pose.pose.pose.orientation

    def commandAction(self, roll, pitch, thrust, yaw_rate, action_time):
        self.twist_msg.linear = Vector3(roll, pitch, thrust)
        self.twist_msg.angular = Vector3(0, 0, yaw_rate)
        data_points = int(self.hz*action_time)
        for i in range(data_points):
            self.pub_twist.publish(self.twist_msg)
            self.rate.sleep()

    def commandThrust(self, thrust): # tested and works
        self.twist_msg.linear = Vector3(0, 0, thrust)
        self.twist_msg.angular = Vector3(0, 0, 0)
        while not rospy.is_shutdown():
            self.pub_twist.publish(self.twist_msg)
            self.rate.sleep()

    def hoverWithFeedback(self, z_ref):
        # self.counter = 0
        # self.time_delay = 10
        self.z_error_new = 0
        self.z_kp = 10000
        self.z_ki = 10
        self.feed_forward = 39000
        while not rospy.is_shutdown():
           
            # self.twist_msg.linear.z = self.feed_forward
            # if self.counter == self.time_delay:

            self.pose_cf = self.getPose('crazyflie2')  
            self.z_actual = self.pose_cf.z
            self.z_error = z_ref - self.z_actual
            # self.z_error_new += self.z_error

            self.z_error_scaled = self.z_error * self.z_kp
            self.twist_msg.linear.z = self.feed_forward + self.z_error_scaled
            self.twist_msg.linear.x = -1 # Trim based on observation
            self.twist_msg.linear.y = 1

            print("The commanded thrust is: {}".format(self.twist_msg.linear.z))
            print("The error is: {}".format(self.z_error_scaled))
            self.pub_twist.publish(self.twist_msg)
    
    def hoverWithBangBang(self, z_ref, circle_radius):
        self.counter = 0
        self.z_feed_forward = 39000
        self.z_kp = 10000
        self.feed_forward = 39000
        self.xy_kp = 10
        self.yaw_kp = -30
        self.x_origin = 0
        self.y_origin = 0
        while not rospy.is_shutdown():
            self.pose_cf = self.getPose('crazyflie2')
            self.x_actual = self.pose_cf.x
            self.y_actual = self.pose_cf.y
            self.z_actual = self.pose_cf.z

            self.z_error = z_ref - self.z_actual
            # self.z_error_new += self.z_error

            self.z_error_scaled = self.z_error * self.z_kp
            self.twist_msg.linear.z = self.feed_forward + self.z_error_scaled
            self.twist_msg.linear.x = -1 # Trim based on observation
            self.twist_msg.linear.y = 1
            
            # Initialize lateral bang bang controller after a time delay
            if self.counter >= 15:
                self.offset = np.sqrt((self.x_actual - self.x_origin)**2 + (self.y_actual - self.y_origin)**2)
                self.theta = np.arctan2((self.y_actual - self.y_origin), (self.x_actual - self.x_origin))
                print("")
                if self.offset > circle_radius:
                    self.twist_msg.linear.x = -1 * self.xy_kp * self.offset * np.cos(self.theta)
                    self.twist_msg.linear.y = self.xy_kp * self.offset * np.sin(self.theta)
                    # print("Lateral controller triggered, offset: {}, theta: {}".format(self.offset, self.theta))
            
            self.quat_actual = self.getPoseQuaternion('crazyflie2')
            self.quat_actual = [self.quat_actual.x, self.quat_actual.y, self.quat_actual.z, self.quat_actual.w]
            R = Rotation.from_quat(self.quat_actual)
            self.global_x = R.apply([1, 0, 0])
            self.yaw_angle = (-1) * np.arctan2(np.cross([1, 0, 0], self.global_x)[2], np.dot(self.global_x, [1, 0, 0]))
            print(self.yaw_angle)
            self.twist_msg.angular.z = self.yaw_kp * self.yaw_angle

            self.pub_twist.publish(self.twist_msg)
            self.counter += 1


if __name__ == "__main__":
    rospy.init_node('test')


    try:
        test1 = Tester()
        # test1.takeOff(0.4) # Using hover msg which uses zdistance

        # Command just thrust
        # test1.commandThrust(10000)

        # Command anything for a certain amount of time
        # roll = 0
        # pitch = 0
        # thrust = 10000
        # yaw_rate = 0
        # time_seconds = 0.3
        # test1.commandAction(roll, pitch, thrust, yaw_rate, time_seconds) 

        # z_ref = 0.5 # command height in meters
        # test1.hoverWithFeedback(z_ref)

        z_ref = 0.5; circle_radius = 0.1; 
        test1.hoverWithBangBang(z_ref, circle_radius)
    
    except Exception as e:
        print(e)
