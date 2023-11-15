# Tools 
Developemnt CLI/ GUI tools

## ComsCLI
Command Line interface for the different tools and functions. 
Provides argument compleation and self documentation. 


## SerialZMQ
Initiaizes device handshake and obtains command map from device.
Reads Serial Port and publishes messages over an IPC socket. 
Writes from message queue to Serial Device based on Protocol. 
``` mermaid
graph LR
a(Device)-->b[Uart/Usb]-->c[pySerial]-->d(decode)-->e[zmqPUBsocket]
```


## Console
Simple Terminal
Subscribes to a ZMQ socket in a threadworker, Usefull for vewing different topics simultanously. 


