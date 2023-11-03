from SerialZMQ import SerialDevice
from logger import getmylogger
import typer
from typing import Optional
from rich import print
import zmq


app = typer.Typer()
ctx = zmq.Context()
log = getmylogger(__name__)
dev = SerialDevice(ctx)


@app.command()
def ports(key : Optional[str] = "usb") -> bool:
    ports = dev.scanUSB(key)
    if len(ports) == 0:
       log.warning("No Ports Found")
       return False
    for p in ports:
        print(f"Found Port: {p.device}, Description: {p.description}")
    return True


@app.command()
def begin(portname : str, baud: Optional[int] = 9600, duration: Optional[int] = 10):

    if dev.connect(portname, baud ):
        dev.readAlive = True
        info()
        try:
            dev._update(duration)
        except KeyboardInterrupt:
            dev.disconnect()


@app.command()
def info():
    info = dev.getInfo()
    print('------------')
    for key, value in info.items():
        print(f'{key} : {value}')
    print('------------')
    



if __name__ == "__main__":
    app()

