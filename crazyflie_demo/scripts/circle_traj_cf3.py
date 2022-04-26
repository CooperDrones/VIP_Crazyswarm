#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
from circle_traj import TrajGenerator
import rospy
import time
#import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from pytz import timezone


if __name__ == '__main__':
    # Generate trajectory
    #wave_traj = StandingWaveGenerator()
    wave_traj = TrajGenerator()
    omega = 1.0 # lower is slower
    num_oscillations = 3
    num_drones = 3
    # traj = wave_traj.genWaveTraj(amplitude, frequency, \
    #     num_oscillations, num_drones)
    

    # Handle discrepancy between military and AM/PM time
    tz = timezone('EST')
    now = datetime.now(tz)
    start_time = rospy.get_param("/crazyflie3/controller/start_time")
    if now.hour > 12:
        global_start = 3600*(float(start_time[11:13]) + 12) + \
            60*float(start_time[14:16]) + float(start_time[17:])
    else:
        global_start = 3600*float(start_time[11:13]) + \
            60*float(start_time[14:16]) + float(start_time[17:])
    t_offset = -3580.0
    global_sync_time = global_start + t_offset 

    # Drone instructions
    z_c = 0.6 # height setpoint
    y_c = 0 # y offset in wave
    x_c = -0.5 # x offset in wave
    yaw_c = 0.0 # should be same for this particular demo
    traj = wave_traj.genCircleTraj(x_c,y_c,0,0,omega,num_oscillations)

    cf3 = CooperativeQuad('crazyflie3')
    cf3.hoverStiff(x_c,y_c, z_c, yaw_c, 0.05, False,
        True, global_sync_time)
    #cf3.trajTrackingStandingWave(traj, z_c, y_c)
    cf3.trajTracking(traj, z_c)
    cf3.hoverStiff(0, 0  , z_c, yaw_c, 0.05)
    cf3.land()