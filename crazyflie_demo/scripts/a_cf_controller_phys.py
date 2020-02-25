#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import pickle
from datetime import datetime

import os
here = os.path.dirname(os.path.abspath(__file__))

# Import crazyflie model modules
import sys
sys.path.append("../model/")
import crazyflie_param as P

plt.ion()

# # Followed this paper, section 3, for all controllers
# # https://arxiv.org/pdf/1608.05786.pdf
class AltitudeControllerPhys:
    def __init__(self, ff=41000.0, kp=11000.0, ki=3500.0, kd=9000.0):
        self.ff = ff # Feedforward term
        self.kp = kp
        self.ki = ki
        self.e_hist = 0.0
        self.e_cap = 1.5
        self.kd = kd
        self.e_prev = 0.0

        self.t_phys1 = 1/30.0
    
    def update(self, z_c, z):
        """
        Off-Board altitude controller from Canada Paper

        Parameters
        ----------
        z_c = z position setpoint
        z =   current z position

        Returns
        -------
        del_omega_cap = PWM signal that maps to commanded thrust
        """
        e = z_c - z
        # print("commanded pos is {}, actual pos is {}".format(z_c, z))
        # self.e_hist += (e * t) # historical error
        if self.e_hist <= self.e_cap: # prevent wind-up
            self.e_hist += (e * self.t_phys1)

        e_der = (e - self.e_prev) / self.t_phys1 # dirty derivative error
        self.e_prev = e
        # print("error: {}, der error: {}, hist error {}".format((e * self.kp), (e_der * self.kd), (self.e_hist * self.ki)))
        del_omega_cap = self.ff + (self.kp * e) + (self.ki * self.e_hist) + (self.kd * e_der)
        # print("del_omega_cap", del_omega_cap)
        return del_omega_cap

class XYControllerPhys:
    def __init__(self, kp=10.0, ki=2.0, cap=15.0):
        self.kp = kp
        self.ki = ki
        self.cap = cap
        self.x_b_prev = 0.0
        self.y_b_prev = 0.0
        self.xe_b_hist = 0.0
        self.ye_b_hist = 0.0

        # TODO: update this to follow parameter file
        self.t_phys2 = 1/30.0

    def update(self, x_c, x, y_c, y, yaw):
        """
        Off-Board XY position controller from Canada Paper

        Parameters
        ----------
        x_c = x position setpoint
        x =   current x position
        y_c = y position setpoint
        y =   current y position
        yaw = current yaw angle

        Returns
        -------
        phi_c = total body-frame x position error which maps to commanded pitch angle 
        theta_c = total body-frame y position error which maps to commanded roll angle
        """
        xe = x_c - x; ye = y_c - y # Get position error
        # print("class implementation\nxe: {}\nye: {}".format(xe, ye))

        x_b = x * np.cos(yaw) + y * np.sin(yaw) # Get x in body frame
        u = (x_b - self.x_b_prev) / self.t_phys2 # u is x-vel in body frame
        self.x_b_prev = x_b # Reset previous val

        y_b = -(x * np.sin(yaw)) + y * np.cos(yaw) # Get y in body frame
        v = (y_b - self.y_b_prev) / self.t_phys2 # v is y-vel in body frame
        self.y_b_prev = y_b # Reset previous val

        # print("class implementation\nu: {}\nv: {}".format(u, v))

        xe_b = xe * np.cos(yaw) + ye * np.sin(yaw) # Get errors in body frame
        ye_b = -(xe * np.sin(yaw)) + ye * np.cos(yaw)

        self.xe_b_hist += ((xe_b - u) * self.t_phys2) # Accumulate and store histroical error
        self.ye_b_hist += ((ye_b - v) * self.t_phys2)

        phi_c = ((xe_b - u) * self.kp) + (self.xe_b_hist * self.ki) # Eq. 3.1.11 and Eq. 3.1.12
        theta_c = ((ye_b - v) * (-self.kp)) + (self.ye_b_hist * (-self.ki))

        # Cap roll (y) and pitch (x) to prevent unstable maneuvers
        if np.abs(phi_c) >= self.cap:
            phi_c =  np.sign(phi_c) * self.cap

        if np.abs(theta_c) >= self.cap:
            theta_c = np.sign(theta_c) * self.cap
        
        return phi_c, theta_c

class XYControllerTrajPhys:
    def __init__(self, kp=100.0, kd=100.0, k_ff = 10.0, cap=20.0):
        self.kp = kp
        self.kd = kd # adding damping
        self.k_ff = k_ff
        self.cap = cap

        self.r_prev = np.array([0.0, 0.0])
        # self.xe_b_hist = 0.0
        # self.ye_b_hist = 0.0

        self.t_phys = 1 / 30.0
        self.g = 9.80665

        # For plotting
        self.time_now = 0.0
        self.time_list = []
        self.x_list = []
        self.x_t_list = []
        self.y_list = []
        self.y_t_list = []
        self.xd_list = []
        self.yd_list = []
        self.xd_t_list = []
        self.yd_t_list = []
    
    def normalize(self, a):
        """
        Return the normal unit vector.

        param a: vector ([float])
        """
        normal = np.empty_like(a)
        normal[0] = -a[1]
        normal[1] = a[0]
        normal = normal / np.linalg.norm(normal)
        return normal

    def update(self, r_t, rd_t, r_t_vect, r, yaw_c, rdd_t, is_pickling=False):
        """
        Off-Board trajectory PID controller

        Parameters
        ----------
        r_t      = traj pos
        rd_t     = traj vel
        r_t_vect = vector from current to next traj point
        r        = actual drone pos
        yaw_c    = yaw setpoint
        rdd_t    = pre-computed feedforward
        is_pickling = save out data into 'cf_data' file

        Returns
        -------
        phi_c = total x position error which maps to commanded pitch angle 
        theta_c = total y position error which maps to commanded roll angle
        """
        # Calculate position component
        t_unit = r_t_vect / np.linalg.norm(r_t_vect)
        n_unit = self.normalize(t_unit)

        print("t_unit is ", t_unit)
        print("n_unit is ", n_unit)

        print("r_t - r is {}".format(r_t - r))

        # only account for cross-track position error
        e_p = np.dot(np.dot((r_t - r), n_unit), n_unit) \
            # + np.dot(np.dot((r_t - r), t_unit), t_unit)

        # Calculate velocity error component
        rd = (r - self.r_prev) / self.t_phys
        self.r_prev = r

        # TODO: make some plots to check velocity smoothness
        # using pickle, tune parameters
        # let Prof. know if we need a filter derivative
        print("r is ", r)
        print("rd is ", rd)
        e_v = rd_t - rd
        # e_v = np.dot(np.dot((rd_t - rd), t_unit), t_unit)

        # Save out data if desired
        if is_pickling:
            self.time_now += self.t_phys
            self.time_list.append(self.time_now)
            self.x_list.append(r[0])
            self.x_t_list.append(r_t[0])

            self.xd_list.append(rd[0])
            self.xd_t_list.append(rd_t[0])

            self.y_list.append(r[1])
            self.y_t_list.append(r_t[1])
            
            self.yd_list.append(rd[1])
            self.yd_t_list.append(rd_t[1])

        # Calculate accel vector and convert to desired angular commands
        print("total position component {}".format(self.kp * e_p))
        print("total velocity component {}".format(self.kd * e_v))
        print("total accelera component {}".format(rdd_t))
        
        rdd_t = self.kp * e_p + self.kd * e_v \
            # + self.k_ff * rdd_t # optional feedforward component
        print("total rdd_t {}".format(rdd_t))

        theta_c = 1.0/self.g * (rdd_t[0] * np.sin(yaw_c) - \
            rdd_t[1] * np.cos(yaw_c)) # equivalent to movement in -y direction
        phi_c   = 1.0/self.g * (rdd_t[0] * np.cos(yaw_c) + \
            rdd_t[1] * np.sin(yaw_c)) # equivalent to movement in +x direction

        # Cap roll (y) and pitch (x) to prevent unstable maneuvers
        if np.abs(phi_c) >= self.cap:
            phi_c =  np.sign(phi_c) * self.cap
        if np.abs(theta_c) >= self.cap:
            theta_c = np.sign(theta_c) * self.cap

        return phi_c, theta_c
    
    def pickleData(self):
        print('Pickling data!')
        data = np.array([self.time_list,
            self.x_list,    
            self.x_t_list,  
            self.xd_list,
            self.xd_t_list,
            self.y_list,    
            self.y_t_list,  
            self.yd_list,
            self.yd_t_list,
        ])

        with open(os.path.join(here, "cf_data"), 'wb') as outfile:
            pickle.dump(data, outfile)

        outfile.close()
        print('Pickling has completed!')

class YawControllerPhys:
    def __init__(self, kp=-20.0):
        self.kp = kp
    
    def update(self, yaw_c, yaw):
        """
        Off-Board global yaw angle controller

        Parameters
        ----------
        yaw_c = yaw angle setpoint
        yaw =   current yaw angle

        Returns
        -------
        psid_c = total yaw angle error which maps to commanded yaw rate
        """
        yawe = yaw_c - yaw
        psid_c = self.kp * yawe
        return psid_c