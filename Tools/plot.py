import sys
from PyQt6 import QtCore
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.animation as animation
import numpy as np

import random
import threading
import zmq
from logger import getmylogger
log = getmylogger(__name__)

class MatplotlibWidget(QWidget):
    def __init__(self, parent=None):
        super(MatplotlibWidget, self).__init__(parent)
        self.topic = "A/"
        self.subAddr = 'ipc://SHARED' 
        # Create a figure and axis for the plot
        self.x_len = 200
        self.y_range = [-1.5,1.5]
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        self.canvas = FigureCanvas(self.fig)

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
        self.ax.set_ylim(self.y_range)
        self.line, = self.ax.plot(self.xs, self.ys)
        self.animation =  animation.FuncAnimation(self.fig,self.animate,fargs=(self.ys,),interval=50,blit=True,cache_frame_data=False)
        
    @QtCore.pyqtSlot(str)
    def _updateData(self, msg):
       yval = float(msg.split(":")[0])
       self.ys.append(yval)

    
    def animate(self, i, ys):
        # Update the plot with new data
        # Limit y list to set number of items
        self.ys = self.ys[-self.x_len:]
        # Update line with new Y values
        self.line.set_ydata(self.ys)
        return self.line,


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Set up the main window
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Real-time Plot with PyQt and Matplotlib')

        # Create the central widget
        central_widget = MatplotlibWidget(self)
        self.setCentralWidget(central_widget)


class ZMQReceiver(QObject):

    socketDataSig = QtCore.pyqtSignal(str)
   
    def __init__(self, topic, subAddr ): 
        super().__init__()
        self.subAddr = subAddr
        self.topic = topic.encode()
        self.context = zmq.Context.instance()
        
    def start(self):
        threading.Thread(target=self._execute, daemon=True).start()

    def _updateFilt(self, topic : str):
        self.topic = topic.encode()
        
    def _execute(self):
        '''Execute Thread'''

        log.info("Started ZMQ Receiver")
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.subAddr)
        self.socket.setsockopt( zmq.SUBSCRIBE, self.topic)
       
        while True:
            try:
                data = self.socket.recv().decode()[2:]
                self.socketDataSig.emit(data)
            except Exception as e:
                log.errror(f"Expeption in Zmq Recviever:{e} ")
                break
        log.info(f"exit ZMQ Thread Sub: {self.topic}")



def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
