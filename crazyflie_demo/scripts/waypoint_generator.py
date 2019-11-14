import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

if __name__ == "__main__":

    # waypoints = np.array([[0, 0, 0.4], 
    # [2, 0, 0.4], 
    # [-2, 0, 0.4],
    # [2, 0, 0.4], 
    # [-2, 0, 0.4],
    # [2, 0, 0.4], 
    # [-2, 0, 0.4]])


    N = 30
    x = np.linspace(0.0, 2*np.pi, N)

    waypoints = np.zeros((N,3))
    for i in range(len(x)):
        waypoints[i, 0] = np.sin(x[i])
        waypoints[i, 1] = np.cos(x[i])
        waypoints[i, 2] = 0.4

    # waypoints = np.zeros((N*2,3))
    # for i in range(len(x)):
    #     waypoints[i, 0] = x[i]
    #     waypoints[i, 1] = np.sqrt(1 - x[i]**2)
    #     waypoints[i, 2] = 0.4
    # for i in range(len(x)):
    #     waypoints[i+len(x), 0] = x[i]
    #     waypoints[i+len(x), 1] = -np.sqrt(1 - x[i]**2)
    #     waypoints[i+len(x), 2] = 0.4


    print(waypoints)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    ax.scatter3D(waypoints[:,0], waypoints[:,1], waypoints[:,2])

    plt.show()