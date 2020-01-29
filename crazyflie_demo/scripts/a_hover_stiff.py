#!/usr/bin/env python
import numpy as np
import math
# import scipy.interpolate as si
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from mpl_toolkits import mplot3d

# Import crazyflie model modules
from a_cf_controller_phys import AltitudeControllerPhys, XYControllerPhys
import sys
sys.path.append("../model/")
from data_plotter import DataPlotter
import crazyflie_param as P

# Import ros specifc modules
import rospy
from geometry_msgs.msg import Twist, Vector3 # twist used in cmd_vel
from vicon_bridge.srv import viconGrabPose

plt.ion() # enable interactive plotting

class CooperativeQuad:
    def __init__(self, cf_name):
        self.cf_name = cf_name
        self.msg = Twist()
        self.hz = 30.0
        self.t_phys = 1/self.hz
        self.rate = rospy.Rate(self.hz)

        self.pub = rospy.Publisher('crazyflie/cmd_vel', Twist, queue_size=0)
        rospy.wait_for_service('/vicon/grab_vicon_pose')
        self.pose_getter = rospy.ServiceProxy('/vicon/grab_vicon_pose', viconGrabPose)

        # # Followed this paper, section 3.1, for PID controller
        # # https://arxiv.org/pdf/1608.05786.pdf
        # # Altitude (z) controller gains and initialization
        # self.z_feed_forward = 41000. # Eq. 3.1.8 - a bit less since we do not use UWB module
        # self.z_kp = 11000. # Table 3.1.3
        # self.z_ki = 3500.
        # self.z_kd = 9000.
        # self.thrust_cap_high = 15000. # TODO add caps for all commands
        # self.thrust_cap_low = -20000.
        # self.ze_cap = 1.5

        # xy controller gains and initialization
        self.x_kp = 10.0 # Table 3.1.3
        self.x_ki = 2.0
        self.y_kp = -10.0
        self.y_ki = -2.0
        self.x_cap = 15.0
        self.y_cap = 15.0

        # Yaw rate controller gains
        self.yaw_kp = -20. # Table 3.1.3

        # Plotter initialization
        start = self.getPose(self.cf_name)
        self.to_plot = np.array([start.position.x, start.position.y, start.position.z])

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
        # x_b_prev = 0.0
        # xe_b_hist = 0.0
        # y_b_prev = 0.0
        # ye_b_hist = 0.0
        pose = self.getPose('crazyflie4')
        yawr = 0.0
        # t = 0.0

        # Initialize classes
        plot = DataPlotter()
        altitude_ctrl_phys = AltitudeControllerPhys()
        xy_ctrl_phys = XYControllerPhys()
        print("after class declarations")

        state = np.array([
            [P.x0],     # 0
            [P.y0],     # 1
            [P.z0],     # 2
            [P.psi0],   # 3
            [P.theta0], # 4
            [P.phi0],   # 5
            [P.u0],     # 6
            [P.v0],     # 7
            [P.w0],     # 8
            [P.r0],     # 9
            [P.q0],     # 10
            [P.p0],     # 11
        ])

        r = np.array([
            [0.0], # x
            [0.0], # y
            [zr], # z
            [0.0], # psi
            [0.0], # theta
            [0.0], # phi
        ])
        
        # t = P.t_start

        # while t < P.t_end:
        #     t_next_plot = t + P.t_plot

        # while t < t_next_plot:
        # for _ in range(100):
        while not rospy.is_shutdown():
            # Get current drone pose
            pose_prev = pose
            pose = self.getPose(self.cf_name)
            if math.isnan(pose.orientation.x): # If nan is thrown, set to last known position
                pose = pose_prev

            ### Altitude controller ###
            z = pose.position.z # Get true z value
            self.msg.linear.z = altitude_ctrl_phys.update(r[2,0], z, self.t_phys)

            ### xy position controller ###
            x = pose.position.x; y = pose.position.y # Get true x and y values
            state[0,0] = x; state[1,0] = y; state[2,0] = z # for plotter

            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            self.msg.linear.x, self.msg.linear.y = xy_ctrl_phys.update(r[0,0], x, r[1,0], y, yaw, self.t_phys)

            # xe = xr - x; ye = yr - y # Get position error

            # # Obtain yaw angle from quaternion
            # quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            # R = Rotation.from_quat(quat)
            # x_global = R.apply([1, 0, 0]) # project to world x-axis
            # yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            # x_b = x * np.cos(yaw) + y * np.sin(yaw) # Get x in body frame
            # u = (x_b - x_b_prev) / self.t_phys # u is x-vel in body frame
            # x_b_prev = x_b # Reset previous val
            # y_b = -(x * np.sin(yaw)) + y * np.cos(yaw) # Get y in body frame
            # v = (y_b - y_b_prev) / self.t_phys # v is y-vel in body frame
            # y_b_prev = y_b # Reset previous val
            # xe_b = xe * np.cos(yaw) + ye * np.sin(yaw) # Get errors in body frame
            # ye_b = -(xe * np.sin(yaw)) + ye * np.cos(yaw)
            # xe_b_hist += ((xe_b - u) * self.t_phys) # Accumulate and store histroical error
            # ye_b_hist += ((ye_b - v) * self.t_phys)
            # xe_b_tot = ((xe_b - u) * self.x_kp) + (xe_b_hist * self.x_ki) # Eq. 3.1.11 and Eq. 3.1.12
            # ye_b_tot = ((ye_b - v) * self.y_kp) + (ye_b_hist * self.y_ki)

            # # Cap roll (y) and pitch (x) to prevent unstable maneuvers
            # if xe_b_tot >= self.x_cap:
            #     xe_b_tot = self.x_cap
            # elif xe_b_tot <= -self.x_cap:
            #     xe_b_tot = -self.x_cap
            # elif ye_b_tot >= self.y_cap:
            #     ye_b_tot = self.y_cap            
            # elif ye_b_tot <= -self.y_cap:
            #     ye_b_tot = -self.y_cap

            # self.msg.linear.x = xe_b_tot
            # self.msg.linear.y = ye_b_tot

            ### yaw-rate controller Eq. 3.1.13 ###
            yawe = yawr - yaw
            yawe_tot = self.yaw_kp * yawe
            self.msg.angular.z = yawe_tot

            ### Goal behavior ###
            offset = 0.05 # additional z boundary to speed tests
            if (x > (xr - goal_r) and x < (xr + goal_r)) and \
                (y > (yr - goal_r) and y < (yr + goal_r)) and \
                (z > (zr - goal_r - offset) and z < (zr + goal_r + offset)):
                print('Found the hover setpoint!')
                # break

            # # Add to plotter
            # to_plot_add = np.array([x, y, z])
            # self.to_plot = np.vstack((self.to_plot, to_plot_add))

            # t += self.t_phys
            self.pub.publish(self.msg)
            self.rate.sleep()

            
        #     plot.update(t, r, state, u)
        #     plt.pause(0.000001)

        # # Keeps the program from closing until the user presses a button.
        # print('Press key to close')
        # plt.waitforbuttonpress()
        # plt.close()

def main():
    rospy.init_node('test')

    try:
        # Initialize drone tester class with dummy loop
        cf1 = CooperativeQuad('crazyflie4')
        cf1.dummyForLoop()

        # Hover at z=0.5, works tested 1/27/2020
        goal_r = 0.01
        cf1.hover(0.0, 0.0, 0.5, goal_r)

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()