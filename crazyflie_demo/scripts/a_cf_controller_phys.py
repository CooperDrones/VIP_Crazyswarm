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

# class XYControllerPhys:
#     def __init__(self, )