/************************************************************************************
 *  @file    twsbComs.cpp
 *  @brief   Implementation of pub-rpc protocol for the TWSB over ZMQ
 *  Single-threaded Event Driven IO loop
 *  Parses in-place the xMacro string mapped to the decoded Publisher ID
 *  Bridges commands send over the ZMQ IPC socket to the TWSB over UART
 *  Provides raw command and topic subscription console to the serial port for debugging
 *  Publishes Timestamped Serialized Telemetry Messages to the ZMQ IPC socket
 * 
 *          ----> | ZMQ SUB | ----> | TWSB UART WRITE | ----> | TWSB |
 *          <---- | ZMQ PUB | <---- | TWSB UART READ  | <---- | TWSB |
 ************************************************************************************/

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <queue>

#include "../common/common.hpp"
#include "../common/coms.hpp"

//*** TWSB Coms Console */
#define NODE_NAME "TWSB"


#define DEFAULT_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUD 115200

// ********* CLI UTILS ************************************************************************* //
void cli_display_help(const std::string& program_name) {
    fmt::print("Usage: {} <serial_port> <baud_rate> \n", program_name);
    fmt::print("Example: {} /dev/ttyUSB0 115200\n", program_name);
}


//@Brief: Configure the serial port non-canonical mode
int config_serial_port(const std::string& port, int baud) {
    fmt::print("[TWSB Coms][INFO] Configuring Serial Port: {} at {} baud\n", port, baud);
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


// ********* SERIAL Implementation Pub-RPC
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
                fmt::print(" >> Error: Invalid Telemetry Message\n");
                coms.telemDecodeState = COMS_TELEM_IDLE;
                break;
            default:
                coms.telemDecodeState = COMS_TELEM_IDLE;
                break;
        }
    }
}


// ********* MAIN ************************************************************************* //
int main(int argc, char* argv[]) {
    // --------------- CLI Parsing ----------------- //
    if (argc < 3) {
        cli_display_help(argv[0]);
        return 1;
    }

    std::string port = argv[1];
    int baud_rate = std::stoi(argv[2]);
    if (port.empty() || baud_rate == 0) {
        cli_display_help(argv[0]);
        return 1;
    }

    // ---------- Communication Setup -------------- //
    Protocol proto = {.sof_byte = '<', .eof_byte = '\n', .delim_byte = ':', .offset = 'A'};
    CommandMsg cmd; // Working command variable



    PubMap pub_map("configs/robotConfig.json", NODE_NAME); 
   
    

    struct SerComs coms;
    /* Note this Node is a transparent bridge between the ZMQ and the TWSB
    // All Parameters and Publishers are of the TWSB MCU as implemented in the Firmware
    // The Config is just loaded to provide a means of maping the publisher id to the
    // string name as required by the ZMQ IPC message */
    
    // --------------- IO Setup ----------------- //
    int serial_fd = config_serial_port(port, baud_rate);
    if (serial_fd == -1) return 1;

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0); // Make console input non-blocking
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); 
    // --------------- ZMQ Setup ----------------- //
    zmq::context_t context(1);
    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, NODE_NAME);
    cmd_subsock.bind("tcp://*:5556");
    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.bind("tcp://*:5555"); 
    // --------------- Console Setup ----------------- //
    fmt::print("[TWSB Coms][INFO] Begin TWSB Console\n");
    zmq::pollitem_t poll_items[] = {
        { nullptr, serial_fd, ZMQ_POLLIN, 0 }, 
        { static_cast<void*>(cmd_subsock), 0, ZMQ_POLLIN, 0 } 
    };
    // -------------- Event Loop ----------------- //
    while (true) {
        int rc = zmq::poll(poll_items, 2, std::chrono::milliseconds(-1));
        if (rc == -1) {
            fmt::print("[{}}] Error: Polling Error\n", NODE_NAME);
            break;
        } 
        if (poll_items[0].revents & ZMQ_POLLIN) { // SerialPort Data Available
            sercoms_grab_telemetry(serial_fd, coms, proto, pub_map);  // read and half parse messages from the serial port
            while (!coms.telemMsgQ.empty()) { // Publish all messages in the 
                struct Message telem_msg = coms.telemMsgQ.front(); // grab the mcu's message from the queue
                coms.telemMsgQ.pop(); 
                telem_msg.topic.append("/");
                telem_msg.topic.append(NODE_NAME); // Append the node name to the topic
                // Send Message Under each Publisher ID over zmq
                coms_publish_tsmp_msg(msg_pubsock,telem_msg);// bridge the message to the ZMQ
                
               
            }
        }
        if (poll_items[1].revents & ZMQ_POLLIN) { // ZMQ Command Message Available 
            coms_receive_asciicmd(cmd_subsock, cmd); // Receive the command message
            if(DEBUG_MODE) {fmt::print("[{}][INFO] Received Command: {}{}\n",NODE_NAME,cmd.topic,cmd.data);}
            // Send the command msg to the TWSB MCU transparently
            write(serial_fd, cmd.data.c_str(), cmd.data.size());
        }
    }

    close(serial_fd);
    return 0;
}