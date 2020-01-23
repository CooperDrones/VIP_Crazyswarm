import matplotlib.pyplot as plt
import sys
import numpy as np

import crazyflie_param as P
# from signal_generator import signal_generator
# from crazyflie_animation import crazyflie_animation
from data_plotter import DataPlotter
from crazyflie_dynamics import CrazyflieDynamics
# from crazyflie_controller import RateController, AttitudeController, ControlMixer, OffBoardController

# instatiate crazyflie, controller, and reference classes
cf = CrazyflieDynamics()

# TODO create controller, crazyflie_animation, signal_generator
# # instatiate the simulation plots and animation
data_plot = DataPlotter()
# animation = crazyflie_animation

if __name__ == "__main__":
    # PWM is sent from controller to motors 0 - 65535
    # RPM is sent from motor to cf dynamics 4070.3 - 21666.4 by Eq. 2.6.1
    u = np.array([
        [0.0],
        [0.0],
        [0.0],
        [0.0],
    ])
    # r = 0.5 # zref value [m]
    # Reference is x,y,z global position and cf yaw angle
    r = np.array([
        [0.0], # x
        [0.0], # y
        [0.5], # z
        [0.0], # psi
        [0.0], # theta 
        [0.0], # phi
    ])
    
    t = P.t_start
    while t < P.t_end:
        t_next_plot = t + P.t_plot # Propagate dynamics at rate Ts
        while t < t_next_plot:
            # u = cf.pwm_to_rpm(u_pwm)
            y = cf.update(u)
            t = t + P.Ts
        # animation.update(cf.state)
        data_plot.update(t, r, cf.state, u)
        plt.pause(0.2)
    
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()
