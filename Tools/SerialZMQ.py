import serial
import serial.tools.list_ports

import time, csv, zmq
from typing import Optional

from utils import getmylogger
from sensorSim import SimSensor

log = getmylogger(__name__)


class SerialDevice():

    def __init__(self):

        self.id = None
        self.pubAddr =  "ipc://SHARED"
        self.ctx = zmq.Context.instance()

        self.port = serial.Serial()

        self.readAlive = None
        self.pubs = None
        self.message = {
            'Topic' : None,
            'Size'  : None,
            'Data'  : None
        }
        self.sensor = SimSensor()

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
        line = self.port.readline().decode(('utf-8')).strip()[:-1] # remove \n
        line = ''.join(char for char in line if char.isprintable()) # remove null chars
        return line
        
    def _placeHolder(self, rate: Optional[float] = 0.1):

        data = self.sensor.generate_data_for_topic()
        time.sleep(rate)
        return data
        
    def _update(self, duration):
        '''Read data from serial device publish to ZMQ SHared Socket'''
        pub = self.ctx.socket(zmq.PUB)
        pub.bind(self.pubAddr)
       
        start_time = time.time()
        try:
            with open("sensordata.csv", "w", newline='' ) as csv_file:
                csv_writer = csv.writer(csv_file)    
                while self.readAlive:
                    runtime = time.time() - start_time
                    if runtime > duration & duration != 0:
                        break
                    if self.port.is_open:
                        msg = self._grabMessage()
                    else:
                        msg = self._placeHolder()
                    if msg != "":
                        pub.send_string(msg)
                        csv_writer.writerow([msg])
                #log.debug(msg)
        except Exception as e:
            log.error("Exeption in Update: ", e)
        finally:
            log.info('Serial Update Stopped')
            log.info(f"Update() Ran for {runtime} seconds")
            log.info("Closing PUB socket")
            #self.readAlive = False
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
            self._stopUpdate()
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


