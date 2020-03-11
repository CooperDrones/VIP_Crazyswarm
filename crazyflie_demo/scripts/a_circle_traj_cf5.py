#!/usr/bin/env python
from a_cooperative_quad import CooperativeQuad
from a_traj_generator import StandingWaveGenerator, TrajGenerator
import rospy
import time
import matplotlib.pyplot as plt
from datetime import datetime
from pytz import timezone

if __name__ == '__main__':
    # Initial Hover start point
    z_c = 0.4
    y_c = 0.0
    x_c = 1.0
    yaw_c = 0.0
    cf5 = CooperativeQuad('crazyflie5')

    traj_gen = TrajGenerator()
    x_center = 0.0; y_center = 0.0
    omega = 1.0
    no_osc = 2.0
    circle_traj = traj_gen.genCircleTraj(x_c, y_c, x_center, y_center, \
        omega, no_osc, CCW=True)

    # Handle discrepancy between military and AM/PM time
    tz = timezone('EST')
    now = datetime.now(tz)
    start_time = rospy.get_param("/crazyflie5/controller/start_time")
    if now.hour > 12:
        global_start = 3600*(float(start_time[11:13]) + 11) + \
            60*float(start_time[14:16]) + float(start_time[17:])
    else:
        global_start = 3600*float(start_time[11:13]) + \
            60*float(start_time[14:16]) + float(start_time[17:])
    t_offset = 10.0
    global_sync_time = global_start + t_offset 

    cf5.hoverStiff(x_c, y_c, z_c, yaw_c, 0.05, is_break=False, \
        is_synchronized=True, global_sync_time=global_sync_time)
    cf5.trajTracking(circle_traj, z_c)
    cf5.hoverStiff(x_c, y_c, z_c, yaw_c, 0.1)
    cf5.land()