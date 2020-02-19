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
            0.0, # y pos [m]
            0.0, # y vel [m/s]
            0.0, # time
        ])

        while t < t_end:
            temp = np.zeros(3,)
            temp[0] = A * np.cos(omega*t)
            temp[1] = A * omega * (-np.sin(omega*t))
            temp[2] = t
            traj = np.vstack((traj,temp))
            t += self.t_phys
        
        if show_plots:
            num_rows = 2; num_cols = 1
            fig, ax = plt.subplots(num_rows, num_cols)

            ax[0].plot(traj[:,2], traj[:,0], c='r', label='pos') # plot pos vs. time
            ax[0].plot(traj[:,2], traj[:,1], c='b', label='vel') # plot vel vs. time
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

def main():
    wave_traj = StandingWaveGenerator()
    frequency = 0.5
    traj = wave_traj.genWaveTraj(0.5, frequency, 1, 3, True)
    print(traj.shape[0])

    plt.show()

if __name__ == "__main__":
    main()