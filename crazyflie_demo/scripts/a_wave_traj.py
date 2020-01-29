import numpy as np
import matplotlib.pyplot as plt

class WaveGenerator:
    def __init__(self, hz=30.0):
        self.hz = hz
        self.t_phys = 1/self.hz

    def genWaveTraj(self, A, omega, no_osc):
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
        
        # print(traj.shape)

        fig, ax = plt.subplots()
        ax.plot(traj[:,2], traj[:,0], c='r', label='pos') # plot pos vs. time
        ax.plot(traj[:,2], traj[:,1], c='b', label='vel') # plot vel vs. time
        ax.legend()


def main():
    wave = WaveGenerator()
    frequency = 0.2
    wave.genWaveTraj(1.0, frequency, 2)
    wave.genWaveTraj(0.5, frequency, 2)

    plt.show()

if __name__ == "__main__":
    main()