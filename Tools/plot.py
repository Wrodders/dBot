import sys
from PyQt6 import QtCore
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.animation as animation
import numpy as np


from utils import getmylogger 
from utils import ZMQReceiver

log = getmylogger(__name__)

plt.style.use('dark_background')


class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.title = "Plot"
        # Set up the main window
        self.setWindowTitle(self.title)
        self.setMinimumSize(300, 300)

        # Create GUI Layout
        self.layout = QGridLayout()
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(self.layout)

        # Create the widget
        plotWidget = MatplotlibWidget(self, )
        self.layout.addWidget(plotWidget)

        self.initSignals()


    def initSignals(self):
        self.signals = [{
            "Name": "",
            "Value": 0,
            "Format": ":x" #value after frist : delim
        }]


class MatplotlibWidget(QWidget):
    def __init__(self, parent=None):
        super(MatplotlibWidget, self).__init__(parent)
        self.topic = ""
        self.subAddr = 'ipc://SHARED' 


        # Create a figure and axis for the plot
        self.x_len = 200
        self.y_range = [-2,2]
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        self.canvas = FigureCanvas(self.fig)
        # Create the ZMQ Receiver Thread
        self.receiver = ZMQReceiver(self.topic, self.subAddr)
        self.receiver.socketDataSig.connect(self._updateData)
        self.receiver.start()

        # Create a layout to hold the plot
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Initialize the plot
        self.xs = list(range(0,self.x_len))
        self.ys = [0] * self.x_len
        self.y2s = [0] * self.x_len
        self.ax.set_ylim(self.y_range)
        self.line, = self.ax.plot(self.xs, self.ys, label = "roll")
        self.line2, = self.ax.plot(self.xs, self.y2s, label = "avg")
        self.ax.legend()
        self.animation =  animation.FuncAnimation(self.fig,self.animate,fargs=(self.ys,self.y2s),interval=16,blit=True,cache_frame_data=False)
        
    @QtCore.pyqtSlot(str)
    def _updateData(self, msg):
        #@Breif: Gets called vis Qt Slot when new data received over ZMQ
        try:
            yval = float(msg.split(":")[0])
            avg = self.moving_average(yval, 2)
            self.ys.append(yval)
            self.y2s.append(avg[0])
        except Exception as e:
            log.error(e)
    
    def animate(self, i, ys, y2s):
        #@Breif: gets called by Matplotlib funcAdimate loop
        # Update the plot with new data
        # Limit y list to set number of items
        self.ys = self.ys[-self.x_len:]
        self.y2s = self.y2s[-self.x_len:]
        # Update line with new Y values
        self.line.set_ydata(self.ys)
        self.line2.set_ydata(self.y2s)
        return self.line, self.line2

    def moving_average(self, data, window_size):
        return np.convolve(data, np.ones(window_size)/window_size, mode='valid')



def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
