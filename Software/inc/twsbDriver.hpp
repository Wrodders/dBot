#include <sys/select.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <chrono>
#include <ctime>
#include <cstring>
#include <iostream>
#include <zmq.hpp>
#include <iomanip>
#include <getopt.h>
#include <vector>
#include <queue>
#include <zmq_addon.hpp>
#include <sys/select.h>

namespace twsb{

#ifdef __linux__
#include "../inc/mcuComs.h"
#elif __APPLE__
#include "../../Firmware/pubrpc/mcuComs.h"
#endif
}

#define DEFAULT_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUD 115200
#define SERIAL_RX_BUFFER_SIZE 1024

// Debugging mode
const bool DEBUG_MODE = false;

/* TWSB Driver

Telemetry Data Message are sent as packets over the serial port framed by the ASCII Protocol 
The ASCII Protocol is defined by MCU_COMS_H accessible over the twsb namespace

The TWSB Driver is responsible for:
    * Parsing the ASCII Protocol Telemetry Data
    * Mapping the MCUs Publisher ID to human-readable topic strings
    * Publishing Timestamped Telemetry Data over ZMQ IPC under the TWSB/SERIAL topic
    * Publishing Telemetry Data over ZMQ IPC
    *
    * Publishing Command Return Messages over ZMQ IPC

*/
enum COMS_DECODE_STATE {COMS_DECODE_IDLE = 0, COMS_DECODE_ID, COMS_DECODE_DATA, COMS_DECODE_ERROR};

struct TelemetryMsg{                // Parsed Telemetry Frame
        std::string topic;
        size_t data_size;           // Size of the ASCII Protocol Payload
        std::vector<uint8_t>  data; // ASCII Protocol Payload
        std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct Coms{
    std::array<uint8_t, SERIAL_RX_BUFFER_SIZE> rxBuffer; // Serial Port Read Buffer
    struct twsb::Protocol serial_protocol;              // ASCII Protocol Configuration

    struct TelemetryMsg telemMsg;                       // Working Telemetry Message
    std::queue<struct TelemetryMsg> telemMsgQ;          // Telemetry Message Queue
    enum COMS_DECODE_STATE decodeState = COMS_DECODE_IDLE; // Decode State
};


//@Brief: Grab the ASCII message frame from the serial port read buffer
//@Description: Bytes are read from serial port and decoded into a message frame
//@Note: If multiple complete frames are available, they are pushed to the queue in sequence
//@Note: If partial frames are available, they can be processed in the next iteration
void grabMsgFrame(uint32_t serialFd, struct Coms &coms, std::queue<struct TelemetryMsg> &telemMsgQueue) {
    ssize_t bytesRead = read(serialFd, &coms.rxBuffer, coms.rxBuffer.size()); // Read the serial ports
    uint8_t idx = 0;
    uint8_t byte;
    while ((bytesRead - idx) > 0) { // iterate through the read data
        byte = coms.rxBuffer[idx++];
        switch(coms.decodeState){
            case COMS_DECODE_IDLE: // Waiting for Start of Frame Byte
                if (byte== coms.serial_protocol.sof_byte) {
                    coms.telemMsg.topic.clear();
                    coms.telemMsg.data.clear();
                    coms.telemMsg.data_size = 0;
                    coms.decodeState = COMS_DECODE_ID;
                }
                break;
            case COMS_DECODE_ID: // Waiting for ID Byte
                coms.telemMsg.topic = twsb::telemetryVarString(static_cast<twsb::TelemetryVars>(byte));
                coms.decodeState = COMS_DECODE_DATA;
                break;
            case COMS_DECODE_DATA: // grab Data Bytes
                if(byte == coms.serial_protocol.sof_byte){ // Error: escape character
                    coms.decodeState = COMS_DECODE_ERROR;
                } 
                else if(byte == coms.serial_protocol.eof_byte){ // Message Frame Complete
                    coms.telemMsgQ.push(coms.telemMsg);         // Add the message frame to the queue
                    coms.decodeState = COMS_DECODE_IDLE;        // Reset and process remaining bytes
                    break;
                }
                else {coms.telemMsg.data.push_back(byte);} // Add the byte to the data buffer
                break;
            case COMS_DECODE_ERROR: // Error State
                coms.decodeState = COMS_DECODE_IDLE;
                break;
            default:
                break;
        };
    }
}   

//@Brief: Configure the serial port non-canonical mode
int configSerialPort(const std::string& port, int baud) {
    if (DEBUG_MODE) std::cerr << "[DEBUG] Configuring serial port: " << port << " with baud: " << baud << std::endl;

    int serial_port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // Non-blocking mode
    if (serial_port_fd == -1) {
        std::cerr << "Error: Unable to open serial port " << port << std::endl;
        return -1;
    }
    struct termios options;
    tcgetattr(serial_port_fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10; // 1 second timeout
    tcsetattr(serial_port_fd, TCSANOW, &options); // Apply the settings

    // Check if the configuration was successful
    if(tcsetattr(serial_port_fd, TCSANOW, &options) != 0) {
        std::cerr << "Error: Unable to configure serial port " << port << std::endl;
        return -1;
    }

    if (DEBUG_MODE) {std::cerr << "[DEBUG] Serial port configured successfully." << std::endl;}
    return serial_port_fd;
}

//@Brief: Format timestamps in human-readable format
std::string formatTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp) {
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tm = *std::localtime(&time_t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count() % 1000000000;
    oss << "." << std::setw(9) << std::setfill('0') << ns;
    return oss.str();
}