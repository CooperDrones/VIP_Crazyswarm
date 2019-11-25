import numpy as np
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import scipy.interpolate as si

def bspline_planning(x, y, sn):
    """
    Path Plannting with B-Spline
    author: Atsushi Sakai (@Atsushi_twi)
    """
    BSN = 2  # B Spline order

    t = range(len(x))
    x_tup = si.splrep(t, x, k=BSN)
    y_tup = si.splrep(t, y, k=BSN)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)
    return rx, ry

def waypoints():
    # way points
    x = np.array([-2.0, -1.0, 0.0, 1.0, 2.0])
    y = np.array([-0.5,  1.5, -1.5, 1.5, -0.5])
    sn = 100  # sampling number

    rx, ry = bspline_planning(x, y, sn)

    waypoints = np.zeros((sn, 3))
    for i in range(len(rx)):
        waypoints[i, 0] = rx[i] # x
        waypoints[i, 1] = ry[i] # y
        waypoints[i, 2] = 0.5 # z

    # print(waypoints.shape)

    # show results
    plt.plot(x, y, '-og', label="Waypoints")
    plt.plot(rx, ry, '-r', label="B-Spline path")
    plt.grid(True)
    plt.legend()
    plt.axis("equal")
    plt.show()

    fig = plt.figure()
    ax = plt.axes(projection='3d')

if __name__ == '__main__':
    # waypoints()

    np.array([0., 0., 0.5])
    np.array([])

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    ax.plot3D(pose.x, pose.y, pose.z)

    plt.show()