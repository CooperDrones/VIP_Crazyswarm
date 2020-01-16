import matplotlib.pyplot as plt
import sys
import numpy as np

import crazyflie_param as P
# from signal_generator import signal_generator
# from crazyflie_animation import crazyflie_animation
from data_plotter import data_plotter
from crazyflie_dynamics import crazyflie_dynamics

# instatiate crazyflie, controller, and reference classes
cf = crazyflie_dynamics()

# TODO create controller, crazyflie_animation, signal_generator
# # instatiate the simulation plots and animation
data_plot = data_plotter()
# animation = crazyflie_animation

if __name__ == "__main__":
    # PWM is 0 - 65535
    # RPM is 4070.3 - 21666.4 Eq. 2.6.1
    # omega is 426.2 - 2268.9
    u = np.array([
        [1000],
        [1000],
        [1000],
        [1000],
    ])
    # r = 0.5 # zref value [m]
    # Reference is x,y,z global position and cf yaw angle
    r = np.array([
        [0.0], # x
        [0.0], # y
        [0.5], # z
        [0.0], # yaw angle
    ])
    
    t = P.t_start
    while t < P.t_end:
        t_next_plot = t + P.t_plot # Propagate dynamics at rate Ts
        while t < t_next_plot:
            # u = ctrl.update(ref_input, cf.state)
            y = cf.update(u)
            t = t + P.Ts
        # animation.update(cf.state)
        data_plot.update(t, r[2], cf.state, u)
        plt.pause(0.2)
    
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()
