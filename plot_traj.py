from matplotlib import pyplot as plt
import time

class Plot:
    """
    Plot real time trajectory of robot's hand
    """
    def __init__(self, xlim, ylim, title: str = '', add_grid=True) -> None:
        self.fig = plt.figure(figsize=[10, 10])
        self.ax = self.fig.add_subplot(1,1,1)
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_title(title)
        plt.grid(add_grid) 
        plt.ion()  #interactive mode on
        self.obsX = []
        self.obsY = []
        self.line = None
        plt.show()

    def update(self, x, y):
        self.obsX.append(x)
        self.obsY.append(y)
        if self.line is None:
            self.line = self.ax.plot(self.obsX, self.obsY, '-g', marker='o', markersize=2)[0]
        self.line.set_xdata(self.obsX)
        self.line.set_ydata(self.obsY)
        # plt.show()

if __name__ == "__main__":
    """Test class Plot"""
    plot = Plot([-10, 100], [-10, 100])
    t0 = time.time()
    while True:
        x = time.time() - t0 - 10
        y = time.time() - t0
        plot.update(x, y)
        plt.pause(0.01)