import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np

plt.ion()  # enable interactive drawing

class dataPlotter:
    def __init__(self):
        # Number of subplots
        self.num_rows = 3
        self.num_cols = 1

        # Create figure and axis handles
        self.fig, self.ax = plt.subplots(self.num_rows, self.num_cols, sharex=True)

        # Instatiate lists to hold the time and data histories
        self.time_history = [] # time
        self.zref_history = [] # reference position z_r
        self.z_history = [] # position z
        self.x_history = [] # position x
        self.y_history = [] # position y

        # Create a handle for every subplot
        self.handle = []
        self.handle.append(myPlot(self.ax[0], ylabel='z[m]', title='CF Data'))
        self.handle.append(myPlot(self.ax[1], ylabel='x[m]'))
        self.handle.append(myPlot(self.ax[2], xlabel='t(s)', ylabel='y[m]'))

    def update(self, t, reference, states, ctrl)
        # Update the time history of all plot variables
        self.time_history.append(t)
        self.zref_history.append(reference)
        self.z_history.append(states.item(2))
        self.x_history.append(states.item(0))
        self.y_history.append(states.item(1))

        # Update the plots with associated handles
        self.handle[0].update(self.time_history, [self.z_history, self.zref_history])
        self.handle[1].update(self.time_history, [self.x_history])
        self.handle[2].update(self.time_history, [self.y_history])
        