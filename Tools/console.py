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
import argparse


from logger import getmylogger

cmdFormat = namedtuple('cmd', ['cmdId', 'cmdDesc' ])

log = getmylogger(__name__)


class ConsoleApp(QWidget):    
    def __init__(self, topic):
        super().__init__()
        self.title = "topic/"+topic
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
        try:
            self.receiver.socket.close()
        except Exception as e:
            log.errro(e)
        log.info(f"Exiting Console: {self.title} ")

    def initUI(self):
        # Set Up Terminal UI
        self.setWindowTitle(self.title)
        self.setMinimumSize(300, 300)

        # Create GUI Layout
        self.layout = QGridLayout()
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.setLayout(self.layout)

        # Create Widgets
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.console.setAcceptRichText(True)
        self.console.setStyleSheet("background-color: black; color: green;")

    
        # Add Widgets to Layout
        self.layout.addWidget(self.console, 0, 0, 4, 4)


    def connectSignals(self):
        '''Connect GUI Signals'''
        pass

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
                data = self.socket.recv().decode()
                self.socketDataSig.emit(data)
            except Exception as e:
                log.errror(f"Expeption in Zmq Recviever:{e} ")
                break
        log.info(f"exit ZMQ Thread Sub: {self.topic}")

        self.socket.close()
