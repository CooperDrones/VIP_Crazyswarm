#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
from a_traj_generator import StandingWaveGenerator
import rospy
import time
import matplotlib.pyplot as plt
from datetime import datetime
from pytz import timezone

if __name__ == '__main__':
    # Generate trajectory
    wave_traj = StandingWaveGenerator()
    frequency = 2.0 # lower is slower
    amplitude = 0.5
    no_oscillations = 4.5
    no_drones = 3
    traj = wave_traj.genWaveTraj(amplitude, frequency, \
        no_oscillations, no_drones)

    # Handle discrepancy between military and AM/PM time
    tz = timezone('EST')
    now = datetime.now(tz)
    start_time = rospy.get_param("/crazyflie4/controller/start_time")
    if now.hour > 12:
        global_start = 3600*(float(start_time[11:13]) + 12) + \
            60*float(start_time[14:16]) + float(start_time[17:])
    else:
        global_start = 3600*float(start_time[11:13]) + \
            60*float(start_time[14:16]) + float(start_time[17:])
    t_offset = 10.0
    global_sync_time = global_start + t_offset 

    # Drone instructions
    z_c = 0.4 # height setpoint
    y_c = 0.5 # x offset in wave
    yaw_c = 0.0 # should be same for this particular demo
    cf5 = CooperativeQuad('crazyflie5')
    cf5.hoverStiff(amplitude, y_c, z_c, yaw_c, 0.05, False,
        True, global_sync_time)
    cf5.trajTrackingStandingWave(traj, z_c, y_c)
    cf5.hoverStiff(-amplitude, y_c, z_c, yaw_c, 0.05)
    cf5.land()