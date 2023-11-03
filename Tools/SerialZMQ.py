import serial
import serial.tools.list_ports
from collections import namedtuple
import zmq

import lorem
import time
from typing import Optional
from logger import getmylogger

log = getmylogger(__name__)




class SerialDevice():

    def __init__(self, context):

        self.id = None
        self.pubAddr =  "ipc://SHARED"
        self.ctx = context

        self.port = serial.Serial()

        self.readAlive = None
        self.pubs = None
        self.message = {
            'Topic' : None,
            'Size'  : None,
            'Data'  : None
        }

    # Internal Functions     
    def _stopUpdate(self):
        '''Stop Read Loop Thread'''

        if self.readAlive:
            log.info("Stoping SerialZMQ Update")
            self.readAlive = False
            return True
        else:
            log.warning("ERROR: SerialZMQ Update Not Running")
            return False
        
    def _grabMessage(self):
        rxbuf = self.port.readline().decode()[:-1]
        return rxbuf
        
    def _placeHolder(self, rate: Optional[float] = 0.1):
        sentance = lorem.sentence()
        time.sleep(rate)
        return sentance
        
    def _update(self, duration):
        '''Read data from serial device publish to ZMQ SHared Socket'''
        pub = self.ctx.socket(zmq.PUB)
        pub.bind(self.pubAddr)
        start_time = time.time()
        try:
            while self.readAlive:
                runtime = time.time() - start_time
                if runtime > duration :
                    break
                msg = self._placeHolder()
                pub.send_string(msg)
                log.debug(msg)

                #time.sleep(0.05)
        except Exception as e:
            log.error("Exeption in Update: ", e)
            
        log.info('Serial Update Stopped')
        log.info(f"Update() Ran for {duration} seconds")
        log.info("Closing PUB socket")
        self.readAlive = False
        pub.close()
        return # exit thread

    # Public Functions
    def scanUSB(self, key) -> list:
        ports = [p for p in serial.tools.list_ports.comports() if key in p.device]
        return ports
        
    def connect(self, portNum, baud) -> bool:
        '''Connect to serial device and start reading'''
        log.info(f"Connecting Device to: {portNum}, At Baud: {baud}")
        if self.port.is_open == True:
            log.error('Connect Error: Serial Port Already Open')
            return False
        
        self.port.port = portNum
        self.port.baudrate = baud
        self.port.timeout = 0.1
        self.port.xonxoff=1

        try:
            self.port.open()
        except serial.SerialException as e:
            log.error("Exeption in Connect: ", e)
        else:
            log.info(f'Connection to {portNum} Succesfull')
            return True
    
    def disconnect(self):
        '''Disconnect from serial device'''
        if self.port.is_open == False:
            log.warning("Disconnect Error: Serial Port Is Already Closed")
            return False
        try:
            self._stopUpdate() # Stop Read Thread
            self.port.close()
        except Exception as e:
            log.error("Exeption in Disconect:", e)
            return False
        else:
            log.info('Serial Port Closed')
            return True
        
    def getInfo(self) -> dict:
        info = {
            "DeviceId": self.id,
            "Port": self.port.port,
            "Baud": self.port.baudrate,
            "Publish": self.pubAddr,
            "Port Status": "open" if self.port.is_open else "closed"
        }
        return info

if __name__ ==  "__main__":

    ctx = zmq.Context()

    print("Starting Serial Device Init")
    dev = SerialDevice('Serial', ctx)
    ports = dev.scanUSB()
    dev.printPorts(ports);
    if len(ports):
        if dev.connect(ports[0].device, 115200):
            dev.readAlive = True
            dev.printInfo()
            try:    
                dev._update()

            except KeyboardInterrupt:
                dev.disconnect()
                