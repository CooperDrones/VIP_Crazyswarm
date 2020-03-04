#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
from a_traj_generator import StandingWaveGenerator, SignalGenerator
import rospy
import time
import matplotlib.pyplot as plt
from datetime import datetime
from pytz import timezone

if __name__ == '__main__':
    # Initial Hover start point
    z_c = 0.4
    y_c = 0.0
    x_c = 0.5
    yaw_c = 0.0
    cf4 = CooperativeQuad('crazyflie4')
    
    circle_traj = SignalGenerator()
    radius = 0.5
    omega = 2.0
    no_osc = 2.0
    phase_shift = 0.0
    traj = circle_traj.genCircleTraj(x_c, y_c, z_c, radius, \
        omega, no_osc, phase_shift)
    
    cf4.hoverStiff(x_c, y_c, z_c, yaw_c, 0.05)
    cf4.trajTracking(traj, z_c)
    cf4.hoverStiff(x_c, y_c, z_c, yaw_c, 0.1)
    cf4.land()