import logging
from PyQt6 import QtCore
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
import threading
import zmq, random, lorem

def getmylogger(name) -> logging.getLogger():
    file_formatter = logging.Formatter('%(asctime)s~%(levelname)s~%(message)s~module:%(module)s~function:%(module)s')
    console_formatter = logging.Formatter('%(levelname)s -- %(message)s')
    
    file_handler = logging.FileHandler("logfile.log", mode='w')
    file_handler.setLevel(logging.WARN)
    file_handler.setFormatter(file_formatter)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(console_formatter)

    logger = logging.getLogger(name)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    logger.setLevel(logging.DEBUG)
    
    return logger

class SimSensor():

    def __init__(self):

        self.topicMap = {
        
            'A' : self.generate_float_data,
            'B' : self.generate_word_data
        }

    def generate_float_data(self):
        return ':'.join(map(str, [random.uniform(0.0, 1.0) for _ in range(5)]))

    def generate_word_data(self):
        sentance = lorem.sentence()
        return sentance

    def generate_data_for_topic(self,):
        topics = list(self.topicMap.keys())
        topic_id = random.choice(topics)
        return topic_id + '/' + self.topicMap[topic_id]() #execute function
 



class ZMQReceiver(QObject):

    socketDataSig = QtCore.pyqtSignal(str)
   
    def __init__(self, topic, subAddr ): 
        super().__init__()
        self.subAddr = subAddr
        self.topic = topic.encode()
        self.context = zmq.Context.instance()
        self.log = getmylogger(__name__)
        
    def start(self):
        threading.Thread(target=self._execute, daemon=True).start()

    def _updateFilt(self, topic : str):
        self.topic = topic.encode()
        
    def _execute(self):
        '''Execute Thread'''

        self.log.info("Started ZMQ Receiver")
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.subAddr)
        self.socket.setsockopt( zmq.SUBSCRIBE, self.topic)
       
        while True:
            try:
                data = self.socket.recv().decode()
                self.socketDataSig.emit(data)
            except Exception as e:
                self.log.errror(f"Expeption in Zmq Recviever:{e} ")
                break
        self.log.info(f"exit ZMQ Thread Sub: {self.topic}")
