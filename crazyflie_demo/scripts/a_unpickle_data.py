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
        # Only using ax right now
        self.ax = ax
        # self.legend = legend
        # self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        # self.line_styles = ['-', '-', '--', '-.', ':']
        # self.line = []
        
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

def main():
    filename = 'cf_data'
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

    handle[0].plotData(time, data[1], 'r', 'actual')
    handle[0].plotData(time, data[2], 'b', 'commanded')

    handle[1].plotData(time, data[3], 'r', 'actual')
    handle[1].plotData(time, data[4], 'b', 'commanded')

    handle[2].plotData(time, data[5], 'r', 'actual')
    handle[2].plotData(time, data[6], 'b', 'commanded')

    handle[3].plotData(time, data[7], 'r', 'actual')
    handle[3].plotData(time, data[8], 'b', 'commanded')
    
    # Save out plot for future referral with unique timestamp
    now = datetime.utcnow()
    plt.savefig('plots/cd_flight_{}.png'.format(now))

    plt.show()

if __name__ == '__main__':
    main()