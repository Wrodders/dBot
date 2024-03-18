# Raspberry PI

## SerialComs

Reads SerialPort when data is available, processes data into messageFrames, parses frames and transmits them. 

Receives subscribed commands to act on services: 

serialDevice services
------
* listPorts()
* connectPort()
* closePort()
----

