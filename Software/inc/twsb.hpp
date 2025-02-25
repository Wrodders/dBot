#ifndef TWSB_H
#define TWSB_H

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <queue>
#include <syslog.h>

#include "../common/common.hpp"
#include "../common/coms.hpp"


#define DEFAULT_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUD 115200

#define NODE_NAME "TWSB"

//@Brief: Configure the serial port non-canonical mode
int config_serial_port(const std::string& port, int baud) {
    int serial_port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); 
    if (serial_port_fd == -1) {
        std::cerr << "Error: Unable to open serial port " << port << std::endl;
        return -1;
    }
    struct termios options;
    tcgetattr(serial_port_fd, &options); // load current settings
    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default: 
            close(serial_port_fd);
            throw std::runtime_error("Unsupported baud rate.");
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit 
    options.c_cflag &= ~CSIZE; // Mask the character size bits
    options.c_oflag &= ~OPOST; // raw output
    options.c_iflag &= ~ICANON; // Non Canonical mode
    options.c_iflag &= ~ECHO;  // No echo
    options.c_iflag &= ~ECHOE; // No erasure
    options.c_iflag &= ~ISIG; // No interpretation of INTR, QUIT and SUSP
    options.c_cflag &= ~CRTSCTS;   // No flow control
    options.c_cflag |= CS8; // 8 data bits
    options.c_cc[VMIN] = 0; // Read doesn't block
    options.c_cc[VTIME] = 10; 

    if (tcsetattr(serial_port_fd, TCSANOW, &options) != 0) {
        close(serial_port_fd);
        std::cerr << "Error: Unable to configure serial port " << port << std::endl;
        return -1;
    }
    return serial_port_fd;
}

inline int get_bytes_available(int fd) {
    int num_bytes;
    ioctl(fd, FIONREAD, &num_bytes);
    return num_bytes;
}


// ----------------- PC SERIAL Implementation Pub-RPC Protocol ----------------- //
enum SERCOMS_DECODE_TELEM { COMS_TELEM_IDLE = 0, COMS_TELEM_ID, COMS_TELEM_DATA, COMS_TELEM_ERROR};

struct SerComs {
    Message _telem_msg_rx;
    std::queue<Message> telemMsgQ;
    SERCOMS_DECODE_TELEM telemDecodeState;
    std::vector<uint8_t> rxBuffer; // Now owned by SerComs

    // Constructor that initializes the vector with a given size.
    SerComs()
        : rxBuffer(SERCOMS_RX_BUFFER_SIZE) {
        telemDecodeState = COMS_TELEM_IDLE;
    }
};
//@Brief: Grab the ASCII message frame from the serial port read buffer
//@Description: Parses inplace teh xMacro string mapped to the decoded Publisher ID
//@Note: Multiple messages are pared and pushed to the message queue
//@Note: Partial Messages are completed and verified in the next read
void sercoms_grab_telemetry(int serialFd, struct SerComs& coms, struct Protocol& proto, 
                            PubMap& pub_map) {
    // read line from serial port until new line 
    int bytesAvailable = get_bytes_available(serialFd);
    bytesAvailable = std::min(bytesAvailable, static_cast<int>(coms.rxBuffer.size())); 
    ssize_t bytesRead = read(serialFd, coms.rxBuffer.data(), bytesAvailable);
    size_t idx = 0; // index into the buffer
    while ((bytesRead - static_cast<ssize_t>(idx)) > 0) {
        uint8_t byte = coms.rxBuffer[idx++];
        switch (coms.telemDecodeState) {
            case COMS_TELEM_IDLE:
                if (byte == proto.sof_byte) {
                    coms._telem_msg_rx.data.clear(); // Clear the data buffer for the new message
                    coms.telemDecodeState = COMS_TELEM_ID;
                }
                break;
            case COMS_TELEM_ID:
                byte = proto_decode_id(proto, byte);
                coms._telem_msg_rx.topic = pub_map.get_topic_name(byte);
                if(coms._telem_msg_rx.topic.empty()) {
                    coms.telemDecodeState = COMS_TELEM_ERROR;
                } else {
                    coms.telemDecodeState = COMS_TELEM_DATA;
                }
                break;
            case COMS_TELEM_DATA:
               if (byte == proto.eof_byte) { // End of Message
                    coms._telem_msg_rx.time_str = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
                    coms.telemMsgQ.push(coms._telem_msg_rx);
                    coms.telemDecodeState = COMS_TELEM_IDLE;
                } else {
                    coms._telem_msg_rx.data.push_back(byte);
                }
                break;
            case COMS_TELEM_ERROR:
                coms.telemDecodeState = COMS_TELEM_IDLE;
                break;
            default:
                coms.telemDecodeState = COMS_TELEM_IDLE;
                break;
        }
    }
}

//*** TWSB Coms Console */
#define NODE_NAME "TWSB"
#endif // TWSB_H