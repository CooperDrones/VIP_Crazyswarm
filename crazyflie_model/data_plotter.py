import matplotlib.pyplot as plt 
from matplotlib.lines import Line2D
import numpy as np
from crazyflie_dynamics import crazyflie_dynamics
import crazyflie_param as P

plt.ion()  # enable interactive drawing

class data_plotter:
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
        self.handle.append(my_plot(self.ax[0], ylabel='z(m)', title='CF Data'))
        self.handle.append(my_plot(self.ax[1], ylabel='x(m)'))
        self.handle.append(my_plot(self.ax[2], xlabel='t(s)', ylabel='y(m)'))

    def update(self, t, reference, states, ctrl):
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

class my_plot:
    def __init__(self, ax, xlabel='', ylabel='', title='', legend=None):
        """
        ax - handle to the axes of the figure
        legend - a tuple of strings that identify the data
        """
        self.legend = legend
        self.ax = ax
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        self.line_styles = ['-', '-', '--', '-.', ':']

        self.line = []

        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True
    
    def update(self, time, data):
        """
        Adds data to the plot
        Time is a list
        Data is a list of lists, each list is a line on the plot
        """
        if self.init == True:
            for i in range(len(data)):
                self.line.append(Line2D(time, data[i],
                    color=self.colors[np.mod(i, len(self.colors) - 1)],
                    ls=self.line_styles[np.mod(i, len(self.line_styles) - 1)],
                    label=self.legend if self.legend != None else None))
                self.ax.add_line(self.line[i])
            self.init = False
            # add legend if one is specified
            if self.legend != None:
                plt.legend(handles=self.line)
        else: # Add new data to the plot
            # Updates the x and y data of each line.
            for i in range(len(self.line)):
                self.line[i].set_xdata(time)
                self.line[i].set_ydata(data[i])

        # Adjusts the axis to fit all of the data
        self.ax.relim()
        self.ax.autoscale()

# Run some tests
if __name__ == "__main__":
    data_plot = data_plotter()
    cf = crazyflie_dynamics()

    # PWM is 0 - 65535
    # RPM is 4070.3 - 21666.4 Eq. 2.6.1
    # omega is 426.2 - 2268.9
    u = np.array([
        [2000],
        [2000],
        [1900],
        [1900],
    ])
    t = 0
    r = 0.5
    
    t = P.t_start
    while t < P.t_end:
        # Propagate dynamics at rate Ts
        t_next_plot = t + P.t_plot
        while t < t_next_plot:
            y = cf.update(u)
            t = t + P.Ts
        # animation.update(cf.state)
        data_plot.update(t, r, cf.state, u)
        plt.pause(0.2)
    
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()