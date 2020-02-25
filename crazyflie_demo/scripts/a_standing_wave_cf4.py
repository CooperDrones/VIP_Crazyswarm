#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
from a_traj_generator import StandingWaveGenerator
import rospy
import time
import matplotlib.pyplot as plt

if __name__ == '__main__':
    wave_traj = StandingWaveGenerator()
    frequency = 2.0 # lower is slower
    amplitude = 1.0
    no_oscillations = 4.5
    no_drones = 3
    traj = wave_traj.genWaveTraj(amplitude, frequency, \
        no_oscillations, no_drones)

    start_time = rospy.get_param("/crazyflie4/controller/start_time")
    global_start = 3600*(float(start_time[11:13]) + 12) + \
        60*float(start_time[14:16]) + float(start_time[17:])
    t_offset = 10.0
    global_sync_time = global_start + t_offset 

    z_c = 0.4 # height setpoint
    cf4 = CooperativeQuad('crazyflie4')
    cf4.hoverStiff(amplitude, 0.0, z_c, 0.0, 0.05, False,
        True, global_sync_time)

    cf4.trajTrackingStandingWave(traj, z_c)
    cf4.hoverStiff(-amplitude, 0.0, z_c, 0.0, 0.05)
    cf4.hoverStiff(-amplitude, 0.0, 0.1, 0.0, 0.1) # land