import numpy as np
import matplotlib.pyplot as plt

class StandingWaveGenerator:
    def __init__(self, hz=30.0):
        self.hz = hz
        self.t_phys = 1/self.hz

    def genWaveTraj(self, A, omega, no_osc, no_drones, show_plots=False):
        """
        Generate oscillation keeping prescibed x position
        and varying y position and velocity

        Parameters
        ----------
        A      = amplitude of wave [m]
        omega  = frequency of wave travel [hz] 
        no_osc = number of oscillations to simulate

        Returns
        -------
        traj = components of pos, vel, at a certain time
        """

        T = (2.0 * np.pi)/omega # Period
        t_end = no_osc * T # simulation run time
        
        t = 0.0 # initialize time

        traj = np.array([
            0.0, # x pos [m]
            0.0, # x vel [m/s]
            0.0, # x acc [m/s**2]
            0.0  # time
        ])

        while t < t_end:
            temp = np.zeros(4,)
            temp[0] = A * np.cos(omega*t)
            temp[1] = A * omega * (-np.sin(omega*t))
            temp[2] = A * omega**2 * (-np.cos(omega*t))
            temp[3] = t
            traj = np.vstack((traj,temp))
            t += self.t_phys
        
        if show_plots:
            num_rows = 2; num_cols = 1
            fig, ax = plt.subplots(num_rows, num_cols)

            ax[0].plot(traj[:,3], traj[:,0], c='r', label='pos') # plot pos vs. time
            ax[0].plot(traj[:,3], traj[:,1], c='b', label='vel') # plot vel vs. time
            ax[0].plot(traj[:,3], traj[:,2], c='g', label='acc') # plot vel vs. time
            ax[0].set_title('Standing Wave Trajectory')
            ax[0].set_ylabel('position [m] and velocty [m/s]')
            ax[0].set_xlabel('time [s]')
            ax[0].legend()

            x = np.linspace(-1, 1, 100)
            y = np.cos(x)
            ax[-1].plot(x, y, c='b')
        
            # Plot the initial state of the drones 
            sep = 2.0 / (no_drones + 1.0) 
            print('sep is', sep)
            init_pos = -1 + sep
            dist = init_pos
            for _ in range(no_drones):
                print('dist is', dist)
                ax[-1].scatter(dist, np.cos(dist), c='r', marker='x')
                dist += sep

            ax[-1].set_title('Initial Position of drones')
            ax[-1].set_ylabel('y [m]')
            ax[-1].set_xlabel('x [m]')
            ax[-1].legend()

        return traj

class TrajGenerator:
    def __init__(self, hz=30.0):
        self.hz = hz
        self.t_phys = 1/self.hz
    
    def genCircleTraj(self, x_c, y_c, x_center, y_center, \
        omega, no_osc, CCW=True, show_plots=False):
        """
        Generate a circular trajectory with form
        [ x, xd, xdd, y, yd, ydd, z, zd, zdd ]

        Parameters:
        -----------
        x_c, y_c, z_c = starting points of hover controller
        r      = radius of circle for trajectory
        omega  = speed of tracking through circle
        no_osc = number of loops
        """

        T = (2.0 * np.pi)/omega # Period
        t_end = no_osc * T # simulation run time
        # print(t_end)

        t = 0.0
        time_list = []

        no_steps = int(t_end/self.t_phys)
        # print(no_steps)

        traj = np.zeros((no_steps, 9))

        r = np.sqrt((x_c - x_center)**2 + (y_c - y_center)**2)
        x_temp = x_c - x_center; y_temp = y_c - y_center 
        ang = -np.arctan2(y_temp, x_temp)
        # print('angle is {}'.format(ang))

        # Flies Drone CCW when viewing from above
        if CCW == False:
            # sign = -1.0
            ang += np.pi

        for idx in range(no_steps):
            x   = r * np.cos(omega * t - ang) + x_center
            xd  = r * omega * -np.sin(omega * t - ang) 
            xdd = r * omega**2 * -np.cos(omega * t - ang)
            y   = r * np.sin(omega * t - ang) + y_center
            yd  = r * omega * np.cos(omega * t - ang) 
            ydd = r * omega**2 * -np.sin(omega * t - ang)
            traj[idx] = np.array([x, xd, xdd, y, yd, ydd, 0.0, 0.0, 0.0])
            time_list.append(t)
            t += self.t_phys

        if show_plots:
            num_rows = 2; num_cols = 1
            fig, ax = plt.subplots(num_rows, num_cols, figsize=(6,12))

            ax[0].plot(traj[:,0], traj[:,3], c='r')
            ax[0].scatter(x_center, y_center, marker='x', c='r', s=100)
            ax[0].scatter(x_c, y_c, marker='x', c='b', s=100, label='drone start')
            # ax[0].plot(traj[:,3], traj[:,1], c='b', label='vel')
            ax[0].set_title('Circle Trajectory')
            ax[0].set_ylabel('y-pos [m]')
            ax[0].set_xlabel('x-pos [m]')
            ax[0].legend()

            ax[1].plot(time_list, traj[:,0], c='r', label='pos')
            ax[1].plot(time_list, traj[:,1], c='b', label='vel')
            ax[1].plot(time_list, traj[:,2], c='g', label='acc')
            ax[1].legend()

        return traj

def plotWaveTraj():
    wave_traj = StandingWaveGenerator()
    frequency = 2.0
    amplitude = 0.25
    no_oscillations = 1.5
    no_drones = 3
    y_c = 0.0
    traj = wave_traj.genWaveTraj(amplitude, frequency, \
        no_oscillations, no_drones, True)
    plt.show()

def plotCircleTraj():
    traj_gen = TrajGenerator()
    x_c = 0.5; y_c = 0.0
    x_center = 0.0; y_center = 0.0
    omega = 1.0
    no_osc = 2.0
    circle_traj = traj_gen.genCircleTraj(x_c, y_c, x_center, y_center, \
        omega, no_osc, CCW=False, show_plots=True)
    plt.show()

if __name__ == "__main__":
    # plotWaveTraj()

    plotCircleTraj()
