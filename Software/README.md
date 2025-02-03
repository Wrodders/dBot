# Software

## Architecture

### TWSB_Driver: 

* Publishes Messages to bot_msg under TWSB/
* Processes Commands from bot_cmd under TWSB/
* Forwards Command packets from bot_cmd:TWSB

This data from serial buffer when available, grabs message frames and pushes to a queue. Multiple messages might be pushed to the queue in a single grab cycle.
Partial messages are filled and validated on the the next grab cycle. 
This process is also responsible for sending messages to the TWSB. Commands are written from a MISO queue 

The received Message queue is processed according to the TWSB/SERIAL protocol. The MCU sends messages with ascii encoded single byte headers. These are mapped to string using xMacros for use with other modules. e.g 
| MCU PUB IDX | PUB_ENUM  | PUB_STR             | 
| ------------| ----------| ------------------- |
| 0           | PUB_ERROR | "TWSB/SERIAL/ERROR" |
| ...
| 4           | PUB_IMU   | "TWSB/SERIAL/IMU"   |

