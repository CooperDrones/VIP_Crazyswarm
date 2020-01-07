import matplotlib.pyplot as plt
import sys

import crazyflie_param as P
# from signal_generator import signal_generator
# from crazyflie_animation import crazyflie_animation
# from data_plotter import data_plotter
from crazyflie_dynamics import crazyflie_dynamics

# instatiate crazyflie, controller, and reference classes
cf = crazyflie_dynamics()

# TODO create data_plotter, crazyflie_animation, and signal_generator
# # instatiate the simulation plots and animation
# data_plot = data_plotter()
# animation = crazyflie_animation

if __name__ = "__main__":
    u = np.array([
        [10000],
        [10000],
        [10000],
        [10000],
    ])
    
    t = P.t_start
    while t < P.t_end:
        # Propagate dynamics at rate Ts
        t_next_plot = t + P.t_plot
        while t < t_next_plot:
            y = cf.update(u)
            t = t + P.Ts
        # animation.update(cf.state)
        # data_plot.update(t, r, pendulum.state, u)
        plt.pause(0.0001)