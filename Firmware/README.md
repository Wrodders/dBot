# Firmware 

## Bare metal peripheral drivers for STM32F401 BlackPill

### Serial

This should generally be used as part of a higher level communication functionality 

* **Polling** 
  *  Blocks on waiting for byte Received, byte Transmitted & byte Match
  * ***serialRead()*** ***serialWrite()*** ***serialReadLine()***

* **ISR**      
  * Reads/Writes data byte by byte into allocated RingBuffer, new data ignored if full  
  * Must be serviced by main loop fast enough to ensure buffer doesn't fill up.
  * ***serialSend()*** blocks if buffer full. TX ISR flushes Ringbuffer by the TXE interrupt
  * ***serialReceive()*** reads n bytes from RX ringbuffer, blocks if empty 
  * ***serialGrab()*** non-blocking reads n bytes from ringbuffer, if empty returns num bytes read


### I2C
 * **Polling** -Reads/Write n bytes from/to address, blocks on waiting for data


## Sensor Drivers

* **MPU6050**  
  * Reads 3-Axis Accel & Gyro Values
  * Accel & Gyro Offset calibration
* **IMU**
  * FIR LowPass Filter
  * Kalman Filter Pitch Roll Yaw **WIP** 
   


## Communications 

Built on top of the serial driver, 



## Control Algorithm 



