import numpy as np
import matplotlib.pyplot as plt

class StandingWaveGenerator:
    def __init__(self, hz=30.0):
        self.hz = hz
        self.t_phys = 1/self.hz

    def genGraph(self,A,t,omega,num_osc,orientation,CCW = True, show_plots=False):
        """
        Generate oscillation keeping prescibed x position
        and varying y position and velocity

        Parameters
        ----------
        A      = amplitude of wave [m]
        omega  = frequency of wave travel [hz] 
        num_osc = number of oscillations to simulate

        Returns
        -------
        traj = components of pos, vel, at a certain time
        """

        T = (2.0 * np.pi)/omega # Period
        t_end = num_osc * T # simulation run time
        
        t = 0.0 # initialize time

        traj = np.array([
            0.0, # x pos [m]
            0.0, # x vel [m/s]
            0.0, # x acc [m/s**2]
            0.0  # time
        ])

        while t < t_end:
            temp = np.zeros(4,)
            temp[0] = A * np.sqrt(np.cos(2*omega*t)) # lemniscate equation for r
            temp[1] = A * -1 * omega * np.sin(2*omega*t) / np.sqrt(np.cos(2*omega*t)) # lemniscate velocity
            temp[2] = A * -1 * omega * (np.sin(2*omega*t)**2 + 2 * np.cos(2*omega*t)**2) / np.cos(2*omega*t)**(3/2) # lemniscate acceleration
            temp[3] = t
            traj = np.vstack((traj,temp))
            t += self.t_phys
        
        if show_plots:
            num_rows = 3; num_cols = 1
            fig, ax = plt.subplots(num_rows, num_cols, sharex=True, figsize=(12,9))

            ax[0].plot(traj[:,3], traj[:,0], c='r') # plot pos vs. time
            ax[1].plot(traj[:,3], traj[:,1], c='b') # plot vel vs. time
            ax[2].plot(traj[:,3], traj[:,2], c='g') # plot accel vs. time
            ax[0].set_title('Standing Wave Trajectory')
            ax[0].set_ylabel('position [m]')
            ax[1].set_ylabel('velocity [m/s]')
            ax[2].set_ylabel('acceleration [m/s^2]')
            ax[2].set_xlabel('time [s]')
            ax[0].grid(True)
            ax[1].grid(True)
            ax[2].grid(True)
            # ax[0].legend()

            # # Plot potential drone positions
            # x = np.linspace(-1, 1, 100)
            # y = np.cos(x)
            # ax[-1].plot(x, y, c='b')
        
            # # Plot the initial state of the drones 
            # sep = 2.0 / (no_drones + 1.0) 
            # print('sep is', sep)
            # init_pos = -1 + sep
            # dist = init_pos
            # for _ in range(no_drones):
            #     print('dist is', dist)
            #     ax[-1].scatter(dist, np.cos(dist), c='r', marker='x')
            #     dist += sep

            # ax[-1].set_title('Initial Position of drones')
            # ax[-1].set_ylabel('y [m]')
            # ax[-1].set_xlabel('x [m]')
            # ax[-1].legend()

        return traj

class TrajGenerator:
    def __init__(self, hz=30.0):
        self.hz = hz
        self.t_phys = 1/self.hz
    
    def genLemniscateTraj(self, A, omega, num_osc, CCW=True, show_plots=False):
        """
        Generate a circular trajectory with form
        [ x, xd, xdd, y, yd, ydd, z, zd, zdd ]

        Parameters:
        -----------
        x_c, y_c, z_c = starting points of hover controller
        r      = radius of circle for trajectory
        omega  = speed of tracos(t)/(1+sin^2(t))cking through circle
        num_osc = number of loops
        """

        T = (2.0 * np.pi)/omega # Period
        t_end = num_osc* T # simulation run time
        # print(t_end)

        t = 0.0
        time_list = []

        no_steps = int(t_end/self.t_phys)
        # print(no_steps)

        traj = np.zeros((no_steps, 9))

        r = np.zeros(3,)
        

        # x_temp = A * np.cos(omega*t)
        # y_temp = A * np.sin(omega*t)
        # ang = -np.arctan2(y_temp, x_temp)
        # print('angle is {}'.format(ang))

        # Flies Drone CCW when viewing from above
        sign = 1.0
        if CCW == False:
            sign = -1.0
        
        for idx in range(no_steps):
            denominator = 1+np.sin(omega*t)**2
            x   = sign* A * np.cos(omega*t)/denominator
            xd  = sign * -A*omega*np.sin(omega*t)* (np.sin(omega*t)**2 + 2*np.cos(omega*t)**2 + 1) / denominator**2
            xdd = sign * -A*omega**2*np.cos(omega*t) * ( 5*np.sin(omega*t)**4 + (6*np.cos(omega*t)**2+4)*np.sin(omega*t) - 2*np.cos(omega*t)**2 - 1) / denominator**3
            y   = sign * x * np.sin(omega*t)
            yd  = sign * (xd * np.sin(omega*t) + x * omega * np.cos(omega*t))
            ydd = sign * (xdd * np.sin(omega*t) + 2*xd*omega*np.cos(omega*t) - x*omega**2*np.sin(omega*t))
            traj[idx] = np.array([x, xd, xdd, y, yd, ydd, 0.0, 0.0, 0.0])
            time_list.append(t)
            t += self.t_phys


        # for idx in range(no_steps):
        #     r[0] = A * np.sqrt(np.cos(2*omega*t)) # lemniscate equation for r
        #     r[1] = -A * omega * np.sin(2*omega*t) / np.sqrt(np.cos(2*omega*t))
        #     r[2] = -A * omega**2 * (np.sin(2*omega*t)**2 + 2 * np.cos(2*omega*t)**2) / np.cos(2*omega*t)**(3/2)
        #     x   = sign * r[0] * np.cos(omega * t)
        #     xd  = sign * r[1] * np.cos(omega * t) + r[0] * omega * -np.sin(omega * t - ang) 
        #     xdd = sign * r[2] * np.cos(omega * t) + 2 * r[1] * r[0] * omega * -np.sin(omega * t - ang) + r[0] * omega**2 * -np.cos(omega * t - ang)
        #     y   = sign * r[0] * np.sin(omega * t)
        #     yd  = sign * r[1] * np.sin(omega * t) + r[0] * omega * np.cos(omega * t - ang) 
        #     ydd = sign * r[2] * np.sin(omega * t) + 2 * r[1] * r[0] * omega * np.cos(omega * t - ang) + r[0] * omega**2 * -np.sin(omega * t - ang)
        #     traj[idx] = np.array([x, xd, xdd, y, yd, ydd, 0.0, 0.0, 0.0])
        #     time_list.append(t)
        #     t += self.t_phys

        # if show_plots:
        #     num_rows = 3; num_cols = 1
        #     fig, ax = plt.subplots(num_rows, num_cols, figsize=(6,12))

        #     ax[0].plot(traj[:,0], traj[:,3], c='r')
        #     ax[0].scatter(x_center, y_center, marker='x', c='r', s=100)
        #     ax[0].scatter(x_c, y_c, marker='x', c='b', s=100, label='drone start')
        #     # ax[0].plot(traj[:,3], traj[:,1], c='b', label='vel')
        #     ax[0].set_title('Circle Trajectory')
        #     ax[0].set_ylabel('y-pos [m]')
        #     ax[0].set_xlabel('x-pos [m]')
        #     ax[0].legend()

        #     ax[1].plot(time_list, traj[:,0], c='r', label='pos')
        #     ax[1].plot(time_list, traj[:,1], c='b', label='vel')
        #     ax[1].plot(time_list, traj[:,2], c='g', label='acc')
        #     ax[1].legend()
        #     ax[1].grid(True)

        #     ax[2].plot(time_list, traj[:,3], c='r', label='pos')
        #     ax[2].plot(time_list, traj[:,4], c='b', label='vel')
        #     ax[2].plot(time_list, traj[:,5], c='g', label='acc')
        #     ax[2].legend()
        #     ax[2].grid(True)

        return traj

def plotWaveTraj():
    wave_traj = StandingWaveGenerator()
    frequency = 1.0 # lower is slower
    amplitude = 0.5
    num_oscillations = 3.0
    num_drones = 3
    y_c = 0.0
    traj = wave_traj.genWaveTraj(amplitude, frequency, \
        num_oscillations, num_drones, True)
    plt.show()

def plotCircleTraj():
    traj_gen = TrajGenerator()
    x_c = 0.5; y_c = 0.0
    x_center = 0.0; y_center = 0.0
    omega = 1.0
    num_osc = 2.0
    circle_traj = traj_gen.genCircleTraj(x_c, y_c, x_center, y_center, \
        omega, num_osc, CCW=False, show_plots=True)
    plt.show()

if __name__ == "__main__":
    # plotWaveTraj()

    plotCircleTraj()
