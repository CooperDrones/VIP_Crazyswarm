#!/usr/bin/env python
import numpy as np
# import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# Import crazyflie model modules
import sys
sys.path.append("../model/")
import crazyflie_param as P


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
        xe_b_tot = total body-frame x position error which maps to commanded yaw angle 
        ye_b_tot = total body-frame y position error which maps to commanded pitch angle
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

        xe_b_tot = ((xe_b - u) * self.kp) + (self.xe_b_hist * self.ki) # Eq. 3.1.11 and Eq. 3.1.12
        ye_b_tot = ((ye_b - v) * (-self.kp)) + (self.ye_b_hist * (-self.ki))

        # Cap roll (y) and pitch (x) to prevent unstable maneuvers
        if xe_b_tot >= self.cap:
            xe_b_tot = self.cap
        elif xe_b_tot <= -self.cap:
            xe_b_tot = -self.cap
        elif ye_b_tot >= self.cap:
            ye_b_tot = self.cap            
        elif ye_b_tot <= -self.cap:
            ye_b_tot = -self.cap
        
        return xe_b_tot, ye_b_tot

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
        yawe_tot = total yaw angle error which maps to commanded yaw rate
        """
        yawe = yaw_c - yaw
        yawe_tot = self.kp * yawe
        return yawe_tot