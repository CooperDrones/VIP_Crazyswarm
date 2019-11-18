#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,Vector3,TransformStamped # twist used in cmd_vel
from crazyflie_driver.msg import Hover # used in cmd_hover commands vel, yaw rate, and hover height
from crazyflie_driver.srv import Takeoff
from std_msgs.msg import Duration
from vicon_bridge.srv import viconGrabPose
import numpy as np
from scipy.spatial.transform import Rotation
import math

if __name__ == "__main__":
    N = 12
    M = 4
    A = np.zeros((N, N))

    g = 9.81 # m/s^2
    m = 0.35 # g
    Ct = 3.1582e-10 # N/rpm^2
    Cd = 7.9379e-10 # Nm/rpm^2
    Ixx = 1.395e-5 # Kg x m^2
    Iyy = 1.436e-5 # Kg x m^2
    Izz = 2.173e-5 # Kg x m^2
    d = 39e-3 # m

    A = np.zeros((N, N))
    A[0, 6] = 1.
    A[1, 7] = 1.
    A[2, 8] = 1.
    A[3, 9] = 1.
    A[4, 10] = 1.
    A[5, 11] = 1.
    A[6, 4] = g
    A[7, 5] = -g

    B = np.zeros((N, M))
    B[8, 0] = 2. * (Ct/m)
    B[8, 1] = 2. * (Ct/m)
    B[8, 2] = 2. * (Ct/m)
    B[8, 3] = 2. * (Ct/m)
    B[9, 0] = -2. * (Cd/Izz)
    B[9, 1] = 2. * (Cd/Izz)
    B[9, 2] = -2. * (Cd/Izz)
    B[9, 3] = 2. * (Cd/Izz)
    B[10, 0] = -np.sqrt(2) * d * (Ct/Iyy)
    B[10, 1] = np.sqrt(2) * d * (Ct/Iyy)
    B[10, 2] = np.sqrt(2) * d * (Ct/Iyy)
    B[10, 3] = -np.sqrt(2) * d * (Ct/Iyy)
    B[11, 0] = -np.sqrt(2) * d * (Ct/Ixx)
    B[11, 1] = -np.sqrt(2) * d * (Ct/Ixx)
    B[11, 2] = np.sqrt(2) * d * (Ct/Ixx)
    B[11, 3] = np.sqrt(2) * d * (Ct/Ixx)

    print(A)
    print(B)