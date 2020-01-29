#!/usr/bin/env python
import numpy as np
# import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# Import crazyflie model modules
import sys
sys.path.append("../model/")
import crazyflie_param as P

class AltitudeControllerPhys:
    def __init__(self, ff=41000.0, kp=11000.0, ki=3500.0, kd=9000.0):
        self.ff = ff # Feedforward term
        self.kp = kp
        self.ki = ki
        self.e_hist = 0.0
        self.e_cap = 1.5
        self.kd = kd
        self.e_prev = 0.0
    
    def update(self, z_c, z, t):
        e = z_c - z
        # print("commanded pos is {}, actual pos is {}".format(z_c, z))
        # self.e_hist += (e * t) # historical error
        if self.e_hist <= self.e_cap: # prevent wind-up
            self.e_hist += (e * t)

        e_der = (e - self.e_prev) / t # dirty derivative error
        self.e_prev = e
        # print("error: {}, der error: {}, hist error {}".format((e * self.kp), (e_der * self.kd), (self.e_hist * self.ki)))
        del_omega_cap = self.ff + (self.kp * e) + (self.ki * self.e_hist) + (self.kd * e_der)
        print("del_omega_cap", del_omega_cap)
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


    def update(self, x_c, x, y_c, y, yaw, t):
        xe = x_c - x; ye = y_c - y # Get position error

        x_b = x * np.cos(yaw) + y * np.sin(yaw) # Get x in body frame
        u = (x_b - self.x_b_prev) / t # u is x-vel in body frame
        x_b_prev = x_b # Reset previous val

        y_b = -(x * np.sin(yaw)) + y * np.cos(yaw) # Get y in body frame
        v = (y_b - self.y_b_prev) / t # v is y-vel in body frame
        y_b_prev = y_b # Reset previous val

        xe_b = xe * np.cos(yaw) + ye * np.sin(yaw) # Get errors in body frame
        ye_b = -(xe * np.sin(yaw)) + ye * np.cos(yaw)

        self.xe_b_hist += ((xe_b - u) * t) # Accumulate and store histroical error
        self.ye_b_hist += ((ye_b - v) * t)

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