'''
Provides a GUI Console Terminal that subsribes to a topic HEAD
Futher filtering of topic can be done in app

'''

import sys

from PyQt6 import QtCore
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *

import zmq
import threading

from collections import namedtuple



cmdFormat = namedtuple('cmd', ['cmdId', 'cmdDesc' ])



class ConsoleApp(QWidget):

    consoleDataSig = QtCore.pyqtSignal(str)
    
    def __init__(self, topic):
        super().__init__()
        self.title = topic
        self.topic = topic
        self.subAddr = 'ipc://SHARED' 

        self.cmdMap = {
            'id': cmdFormat(1, "get device info"),
            'listcmd': cmdFormat(2, 'get device cmd map')
        } # Used to map command names to hashed cmd id's

        self.initUI()   
        self.connectSignals()

        # Create ZMQ Receiver
        self.receiver = ZMQReceiver(self.topic, self.subAddr)
        self.receiver.socketDataSig.connect(self.updateConsole)
        self.receiver.start()

    def exitHandler(self):
        self.receiver.socket.close()
        print("Exiting Console: ", self.title)

    def initUI(self):
        # Set Up Terminal UI
        self.setWindowTitle(self.title)
        self.setMinimumSize(300, 300)

        # Create GUI Layout
        self.layout = QGridLayout()
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(self.layout)

        # Create Widgets
        self.filterBtn = QPushButton("Filter")
        self.prefBtn = QPushButton("Preferences")

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.console.setAcceptRichText(True)
        self.console.setStyleSheet("background-color: black; color: green;")
        self.input = QLineEdit()
        self.sendBtn = QPushButton("Send")
        
        # Add Widgets to Layout
        self.layout.addWidget(self.filterBtn, 0, 0)
        self.layout.addWidget(self.prefBtn, 0, 1)
        self.layout.addWidget(self.console, 1, 0, 4, 4)
        self.layout.addWidget(self.input, 5, 0,1, 3)
        self.layout.addWidget(self.sendBtn,5, 3)

    def connectSignals(self):
        '''Connect GUI Signals'''
        self.sendBtn.clicked.connect(self.sendCmd)
        self.filterBtn.clicked.connect(self.editFilter)
        self.prefBtn.clicked.connect(self.editPref)

    def clearConsole(self):
        '''Clear Console'''
        self.console.clear()

    def start(self):
        '''Start Console ZMQ Stream'''
        self.receiver.start()
        self.show()

    @QtCore.pyqtSlot(str) 
    def updateConsole(self, data):
        '''Update Console with new data'''
        if self.console.document().lineCount() > 1000:
            self.console.clear()
        self.console.append(data)


    # Function Buttons
    def editFilter(self):
        #placehodler for filter implemtnation 
        pass

    def editPref(self):
        #placeholder for preferences implementattion
        pass


    def sendCmd(self):
        cmdStr = self.input.text()
        self.input.clear()
        self.console.append(cmdStr)
        #place Holder to parsing mapping and sending command


    # helper functions

    def textFormater(self)->str:
        # Formats text and 
        pass




class ZMQReceiver(QObject):

    socketDataSig = QtCore.pyqtSignal(str)

    def __init__(self, topic, subAddr ): 
        super().__init__()
        self.subAddr = subAddr
        self.topic = topic.encode()
        self.context = zmq.Context.instance()
        
    def start(self):
        threading.Thread(target=self._execute, daemon=True).start()
        
    def _execute(self):
        '''Execute Thread'''

        print("Started ZMQ Receiver")
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.subAddr)
        self.socket.setsockopt( zmq.SUBSCRIBE, b"")
       
        while True:
            try:
                data = self.socket.recv().decode()
                self.socketDataSig.emit(data)
            except Exception as e:
                print("Expeption in Zmq Recviever: ")
                print(e)
                break
        print("exit ZMQ Thread")

        self.socket.close()


if __name__ == "__main__":

        # Create GUI
    app = QApplication(sys.argv)
    window = ConsoleApp("serial", )
    app.lastWindowClosed.connect(window.exitHandler)
   
    window.show()
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        window.exitHandler()