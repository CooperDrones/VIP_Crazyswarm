#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
from lemniscate_traj import TrajGenerator
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
    omega = 1 # lower is slower
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
    t_offset = -3590
    global_sync_time = global_start + t_offset 

    # Drone instructions
    z_c = 0.75 # height setpoint
    amp = .75 # amplitude in wave
    yaw_c = 0.0 # should be same for this particular demo

    #radi = 0.2, center = (0, -0.2)
    traj1 = wave_traj.genLemniscateTraj(amp,omega,num_oscillations,True,False)
    #radi = 0.2, center = (0, 0.2)

    cf3 = CooperativeQuad('crazyflie3')
    y_c = 0.0
    x_c = amp
    cf3.hoverStiff(x_c,y_c, z_c, yaw_c, 0.05, False,
        True, global_sync_time)
    #cf3.trajTrackingStandingWave(traj, z_c, y_c)
    cf3.trajTracking(traj1,z_c)
    
    cf3.hoverStiff(0, 0 , z_c, yaw_c, 0.05)
    
    cf3.land()