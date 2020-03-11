import pickle
import matplotlib.pyplot as plt
from datetime import datetime

# TODO: pass in gains and other important information to better 
# track specific flight result to saved plot
# add geneated traj from a_traj_generator??

class MyPlot:
    def __init__(self, ax, xlabel='', ylabel='', title='', legend=None):
        """
        ax - handle to the axes of the figure
        legend - a tuple of strings that identify the data
        """
        self.ax = ax
        
        # Configure the axes
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel(xlabel)
        self.ax.set_title(title)
        self.ax.grid(True)

        # Keeps track of initialization
        self.init = True

    def plotData(self, time, data, color, label=''):
        if self.init:
            self.ax.plot(time, data, c=color, label=label)
        
        self.ax.legend(loc="best")
        self.ax.relim()
        self.ax.autoscale()

def traj():
    filename = 'data_traj'
    infile = open(filename,'rb')
    data = pickle.load(infile)
    infile.close()

    time =  data[0]

    # Determine how many plots are desired
    rows = 4; columns = 1
    fig, ax = plt.subplots(rows, columns, sharex=True, figsize=(12,12))

    # Create a handle for each plot
    handle = []
    handle.append(MyPlot(ax[0], ylabel='x [m]', title='CF Flight Data'))
    handle.append(MyPlot(ax[1], ylabel='xd [m/s]'))
    handle.append(MyPlot(ax[2], ylabel='y [m]'))
    handle.append(MyPlot(ax[3], xlabel='t(s)', ylabel='yd [m/s]'))

    handle[0].plotData(time[1:], data[1][1:], 'r', 'actual')
    handle[0].plotData(time[1:], data[2][1:], 'b', 'commanded')

    handle[1].plotData(time[1:], data[3][1:], 'r', 'actual')
    handle[1].plotData(time[1:], data[4][1:], 'b', 'commanded')

    handle[2].plotData(time[1:], data[5][1:], 'r', 'actual')
    handle[2].plotData(time[1:], data[6][1:], 'b', 'commanded')

    handle[3].plotData(time[1:], data[7][1:], 'r', 'actual')
    handle[3].plotData(time[1:], data[8][1:], 'b', 'commanded')
    
    # Save out plot for future referral with unique timestamp
    now = datetime.utcnow()
    plt.savefig('plots/cd_flight_{}.png'.format(now))

    plt.show()

def trajXY():
    filename = 'data_traj'
    infile = open(filename,'rb')
    data = pickle.load(infile)
    infile.close()

    time =  data[0]

    plt.figure()
    plt.plot(data[1][1:], data[5][1:], c='r', label='acatual')
    plt.plot(data[2][1:], data[6][1:], c='b', label='commanded')
    plt.title('Circular Trajectory XY Data')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.legend(loc='best')
    plt.grid(True)

    # Save out plot for future referral with unique timestamp
    now = datetime.utcnow()
    plt.savefig('plots/cf_flight_{}.png'.format(now))

    plt.show()

def hover(component, var):
    filename = 'data_' + component
    infile = open(filename,'rb')
    data = pickle.load(infile)
    infile.close()

    time =  data[0]
    if component == 'altitude':
        data[1] = data[1] - 0.4
        data[2] = data[2] - 0.4

    print(data[0])
    print(data[1])
    print(data[2])

    plt.plot(time[1:], data[1][1:], c='r', label='acatual')
    plt.plot(time[1:], data[2][1:], c='b', label='commanded')
    plt.title('CF Yaw Data'.format(component))
    plt.xlabel('time [s]')
    plt.ylabel('{}'.format(var))
    plt.legend(loc='best')
    plt.grid(True)

    if component == 'yaw':
        plt.ylim((0.0, 1.2))

    # Save out plot for future referral with unique timestamp
    now = datetime.utcnow()
    plt.savefig('plots/cf_flight_{}.png'.format(now))

    plt.show()

def hoverXY(var=None):
    filename = 'data_xy'
    infile = open(filename,'rb')
    data = pickle.load(infile)
    infile.close()

    time =  data[0]

    if var == 'x':
        plt.figure()
        plt.plot(time[1:], data[1][1:], c='r', label='acatual')
        plt.plot(time[1:], data[2][1:], c='b', label='commanded')
        plt.title('CF X Data')
        plt.xlabel('time [s]')
        plt.ylabel('x [m]')
        plt.legend(loc='lower right')
        plt.grid(True)

    if var == 'y':
        plt.figure()
        plt.plot(time[1:], data[3][1:], c='r', label='acatual')
        plt.plot(time[1:], data[4][1:], c='b', label='commanded')
        plt.title('CF Y Data')
        plt.xlabel('time [s]')
        plt.ylabel('y [m]')
        plt.legend(loc='best')
        plt.grid(True)

    # Save out plot for future referral with unique timestamp
    now = datetime.utcnow()
    plt.savefig('plots/cf_flight_{}.png'.format(now))

    plt.show()

if __name__ == '__main__':
    # traj()
    trajXY()
    # hover('altitude', 'z [m]')
    # hover('yaw', 'psi [deg]')
    # hoverXY(var='x')
    # hoverXY(var='y')
