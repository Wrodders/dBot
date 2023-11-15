from SerialZMQ import SerialDevice
from console import ConsoleApp

from logger import getmylogger
import typer
from typing import Optional
from rich import print
import zmq
import sys
import subprocess

from PyQt6 import QtCore
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *

app = typer.Typer()
ctx = zmq.Context()
log = getmylogger(__name__)
dev = SerialDevice()


@app.command()
def streamport(portname :Optional [str] = None, baud: Optional[int] = 9600, duration: Optional[int] = 60):
    """
    Scans or Opens Serial port, publishes data to socket
    """
    log.info("Starting Serial Device Stream")
    dev = SerialDevice()
    if portname == None:
        ports = dev.scanUSB("usb")
        if len(ports) > 0:
            for p in ports:
                log.info(f"Found Port: {p.device}, Description: {p.description}")
            dev.connect(ports[0].device, 115200)
        else:
            log.warning("No Ports Found")
    dev.readAlive = True
    for key, value in dev.getInfo().items():
        log.info(f'{key} : {value}')
    log.info(f"Running Update for {duration} seconds")
    try:    
        dev._update(duration)
    except KeyboardInterrupt:
        dev.disconnect()


@app.command()
def substream(topic: Optional[str] = "", output : Optional[str] = None):
    """
    Subscribes and filters a data socket
    """
    if output == None:
        sub = ctx.socket(zmq.SUB)
        sub.connect("ipc://SHARED")
        sub.setsockopt( zmq.SUBSCRIBE, topic.encode())
        log.info("Begin Sub Stream")
        try:
            while True:
                try:
                    data = sub.recv().decode()
                    print(data) # output
                except Exception as e:
                    log.error(f"Expeption in testSub: {e}")
                    break
            log.info("Exit test SUB")
            sub.close()
        except KeyboardInterrupt:
            log.info("Exit text sub ")
    elif output == "console":
        try:
            #subprocess.run(["python3", "console.py", topic])

            while True:
                app = QApplication(sys.argv)
                window = ConsoleApp(topic)
                app.lastWindowClosed.connect(window.exitHandler)
                window.show()
                try:
                    sys.exit(app.exec())
                except KeyboardInterrupt:
                    window.exitHandler()

        except Exception as e:
            log.error(e)
        finally:
            raise typer.Exit()   






if __name__ == "__main__":
    app()

