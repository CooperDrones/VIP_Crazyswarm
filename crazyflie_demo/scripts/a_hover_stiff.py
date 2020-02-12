#!/usr/bin/env python
import numpy as np
import math
# import scipy.interpolate as si
# import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from mpl_toolkits import mplot3d

# Import crazyflie model modules
from a_cf_controller_phys import AltitudeControllerPhys, XYControllerPhys, YawControllerPhys
import sys
sys.path.append("../model/")
# from data_plotter import DataPlotter
import crazyflie_param as P

# Import ros specifc modules
import rospy
from geometry_msgs.msg import Twist, Vector3 # twist used in cmd_vel
from vicon_bridge.srv import viconGrabPose

# plt.ion() # enable interactive plotting

class CooperativeQuad:
    def __init__(self, cf_name):
        rospy.init_node('test', anonymous=True)
        self.cf_name = cf_name
        self.msg = Twist()
        self.hz = 30.0
        self.t_phys = 1/self.hz # TODO make P.t_phys import
        self.rate = rospy.Rate(self.hz)

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)

        # # Followed this paper, section 3.1, for PID controller
        # # https://arxiv.org/pdf/1608.05786.pdf

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

    def hoverStiff(self, x_c, y_c, z_c, yaw_c, goal_r):
        """
        Hovers the drone to an accurate global setpoint
        Drone will stay at setpoint until other function is called
        Stiff refers to optimization for global positional accuracy

        Parameters
        ----------
        x_c, y_c, z_c, yaw_c = reference setpoints
        goal_r = bounding radius for when drone is "close enough" to commanded setpoint
        """
        
        print('Start hover controller')
        # self.listener.waitForTransform(self.worldFrame, self.frame, rospy.Time(), rospy.Duration(5.0)) # addition
        
        pose = self.getPose(self.cf_name) # For vicon nan handler
        # plot = DataPlotter()

        # Initialize required hover controllers
        altitude_ctrl_phys = AltitudeControllerPhys()
        xy_ctrl_phys = XYControllerPhys()
        yaw_ctrl_phys = YawControllerPhys()
        print("after class declarations")

        # state = np.array([
        #     [P.x0],     # 0
        #     [P.y0],     # 1
        #     [P.z0],     # 2
        #     [P.psi0],   # 3
        #     [P.theta0], # 4
        #     [P.phi0],   # 5
        #     [P.u0],     # 6
        #     [P.v0],     # 7vicon_object
        #     [P.p0],     # 11
        # ])

        # for plotter
        r = np.array([
            [x_c], # x
            [y_c], # y
            [z_c], # z
            [yaw_c], # psi (yaw)
        ])
        
        # t = P.t_start

        # while t < P.t_end:
        #     t_next_plot = t + P.t_plot

        # while t < t_next_plot:
        # for _ in range(100):
        while not rospy.is_shutdown():
            # print("in while loop")
            pose_prev = pose
            pose = self.getPose(self.cf_name) # get current pose
            if math.isnan(pose.orientation.x): # handle nans by setting to last known position
                pose = pose_prev
            x = pose.position.x; y = pose.position.y; z = pose.position.z
            
            # Obtain yaw angle from quaternion  
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            self.msg.linear.z = altitude_ctrl_phys.update(z_c, z)
            self.msg.linear.x, self.msg.linear.y = xy_ctrl_phys.update(x_c, x, y_c, y, yaw)
            self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)

            ### Goal behavior ###
            offset = 0.05 # additional z boundary to speed tests
            if (x > (x_c - goal_r) and x < (x_c + goal_r)) and \
                (y > (y_c - goal_r) and y < (y_c + goal_r)) and \
                (z > (z_c - goal_r - offset) and z < (z_c + goal_r + offset)):
                print('Found the hover setpoint!')
                break

            # state[0,0] = x; state[1,0] = y; state[2,0] = z # for plotter

            # t += self.t_phys
            self.pub.publish(self.msg)
            self.rate.sleep()

            
        #     plot.update(t, r, state, u)
        #     plt.pause(0.000001)

        # # Keeps the program from closing until the user presses a button.
        # print('Press key to close')
        # plt.waitforbuttonpress()
        # plt.close()

    def land(self):
        print("Land function called")
        self.hoverStiff(0.0, 0.0, 0.2, 0.0, 0.075)


def main():
    

    try:
        # Initialize drone control class with arg matching vicon object name
        cf1 = CooperativeQuad('crazyflie3')
        cf1.dummyForLoop()

        # Hover at z=0.5, works tested 1/27/2020
        goal_r = 0.1
        cf1.hoverStiff(0.0, 0.0, 0.5, 0.0, goal_r)
        cf1.land()

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()