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
    no_oscillations = 1.5
    no_drones = 3
    traj = wave_traj.genWaveTraj(amplitude, frequency, \
        no_oscillations, no_drones)

    z_c = 0.3 # height setpoint
    cf4 = CooperativeQuad('crazyflie4')
    cf4.hoverStiff(amplitude, 0.0, z_c, 0.0, 0.05)
    cf4.trajTrackingStandingWave(traj, z_c)
    cf4.hoverStiff(-amplitude, 0.0, z_c, 0.0, 0.05)
    cf4.land()