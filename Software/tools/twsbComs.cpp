#include <fmt/format.h>
#include <zmq.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <chrono>
#include <ctime>
#include <queue>
#include <array>
#include <unordered_set>
#include <zmq.hpp>

namespace twsb {
#ifdef __linux__
#include "../inc/mcuComs.h"
#elif __APPLE__
#include "../../Firmware/pubrpc/mcuComs.h"
#endif
}

//*** TWSB Coms Console */

#define COMMAND_TOPIC "SERIAL"
#define COMMAND_SOCKET_ADDRESS "ipc:///tmp/botcmds"
#define TELEMETRY_SOCKET_ADDRESS "ipc:///tmp/botmsgs"
#define TELEMETRY_TOPIC "TWSB"
#define DEFAULT_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUD 115200
const int SERIAL_RX_BUFFER_SIZE = 1024;

// ********* CLI UTILS ************************************************************************* //
void display_usage(const std::string& program_name) {
    fmt::print("Usage: {} <serial_port> <baud_rate> <topic1> <topic2> ...\n", program_name);
    fmt::print("Example: {} /dev/ttyUSB0 115200 IMU STATE\n", program_name);
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

enum COMS_DECODE_STATE { COMS_DECODE_IDLE = 0, COMS_DECODE_ID, COMS_DECODE_DATA, COMS_DECODE_ERROR };

struct TelemetryMsg {
    std::string topic;
    size_t data_size;
    std::string data;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct Coms {
    std::vector<uint8_t> rxBuffer = std::vector<uint8_t>(SERIAL_RX_BUFFER_SIZE);
    twsb::Protocol serial_protocol;
    TelemetryMsg telemMsg;
    std::queue<TelemetryMsg> telemMsgQ;
    COMS_DECODE_STATE decodeState;

    // Explicit constructor to ensure serial_protocol is initialized
    Coms() : serial_protocol{
                .sof_byte = '<', 
                .eof_byte = '\n', 
                .delim_byte = ':', 
                .max_msg_data_size = 512, 
                .max_msg_frame_size = 516}, 
                decodeState(COMS_DECODE_IDLE) {}
};

std::string packCommand(const std::string& cmd, uint8_t sof_byte, uint8_t eof_byte) {
    std::string packed_cmd;
    // SOF | DATA | EOF
    packed_cmd.push_back(sof_byte);
    packed_cmd.append(cmd);
    packed_cmd.push_back(eof_byte);
    return packed_cmd;
}

//@Brief: Grab the ASCII message frame from the serial port read buffer
void grabMsgFrame(int serialFd, Coms& coms ) {
    ssize_t bytesRead = read(serialFd, coms.rxBuffer.data(), coms.rxBuffer.size());
    size_t idx = 0;
    while ((bytesRead - static_cast<ssize_t>(idx)) > 0) {
        uint8_t byte = coms.rxBuffer[idx++];
        switch (coms.decodeState) {
            case COMS_DECODE_IDLE:
                if (byte == coms.serial_protocol.sof_byte) {
                    coms.telemMsg = {};  // Reset message
                    coms.decodeState = COMS_DECODE_ID;
                }
                break;
            case COMS_DECODE_ID:
                byte = twsb::comsDecodeID_(byte);
                coms.telemMsg.topic = twsb::publisherString(static_cast<twsb::Publishers>(byte));
                coms.decodeState = COMS_DECODE_DATA;
                break;
            case COMS_DECODE_DATA:
                if (byte == coms.serial_protocol.sof_byte) {
                    coms.decodeState = COMS_DECODE_ERROR;
                } else if (byte == coms.serial_protocol.eof_byte) {
                    coms.telemMsg.timestamp = std::chrono::system_clock::now();
                    coms.telemMsg.data_size = coms.telemMsg.data.size();
                    coms.telemMsgQ.push(coms.telemMsg);
                    coms.decodeState = COMS_DECODE_IDLE;
                } else {
                    coms.telemMsg.data.push_back(byte);
                }
                break;
            case COMS_DECODE_ERROR:
                coms.decodeState = COMS_DECODE_IDLE;
                break;
        }
    }
}

//@Brief: Configure the serial port non-canonical mode
int configSerialPort(const std::string& port, int baud) {
    fmt::print("[TWSB Coms][INFO] Configuring Serial Port: {} at {} baud\n", port, baud);
    int serial_port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port_fd == -1) {
        std::cerr << "Error: Unable to open serial port " << port << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(serial_port_fd, &options);
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
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    if (tcsetattr(serial_port_fd, TCSANOW, &options) != 0) {
        close(serial_port_fd);
        std::cerr << "Error: Unable to configure serial port " << port << std::endl;
        return -1;
    }

    return serial_port_fd;
}


//@Brief: Forward the telemetry message to the ZMQ publisher socket
//@Note: The message is packed as according to the delimeter in the protocol: <topic><data><timestamp>
void publishTimeStampedMsg(zmq::socket_t& pubSocket, struct Coms& coms, const TelemetryMsg& msg) {
    std::string packed_msg = msg.data + std::string(1, coms.serial_protocol.delim_byte) + std::to_string(msg.timestamp.time_since_epoch().count());
    zmq::message_t topic(msg.topic.size());
    memcpy(topic.data(), msg.topic.data(), msg.topic.size());
    zmq::message_t data(packed_msg.size()); 
    memcpy(data.data(), packed_msg.data(), packed_msg.size());
    pubSocket.send(topic, zmq::send_flags::sndmore);
    pubSocket.send(data, zmq::send_flags::none);
}


void displayMsgConsole(const TelemetryMsg& msg) {
    fmt::print("[{}] {}: {}\n", formatTimestamp(msg.timestamp), msg.topic, std::string(msg.data.begin(), msg.data.end()));
}

// ********* MAIN ************************************************************************* //
int main(int argc, char* argv[]) {
    if (argc < 4) {
        display_usage(argv[0]);
        return 1;
    }

    std::string port = argv[1];
    int baud_rate = std::stoi(argv[2]);
    // store console topics in map 
    std::unordered_set<std::string> console_topics;
    for (int i = 3; i < argc; i++) {
        console_topics.insert(argv[i]);
    }

    zmq::context_t context(1);
    zmq::socket_t subSocket = zmq::socket_t(context, zmq::socket_type::sub);
    subSocket.set(zmq::sockopt::linger, 0);
    subSocket.set(zmq::sockopt::subscribe, COMMAND_TOPIC);
    subSocket.connect(COMMAND_SOCKET_ADDRESS);
    
    int subSocketFd = subSocket.get(zmq::sockopt::fd);


    zmq::socket_t pubSocket = zmq::socket_t(context, zmq::socket_type::pub);
    pubSocket.set(zmq::sockopt::linger, 0);
    pubSocket.bind(TELEMETRY_SOCKET_ADDRESS);

    Coms coms;

    int serial_fd = configSerialPort(port, baud_rate);
    if (serial_fd == -1) return 1;

    fd_set read_fds;

    fmt::print("[TWSB Coms][INFO] Begin TWSB Console \n");
    fmt::print("[TWSB Coms][INFO] Console Topic Filter: ");
    for (const auto& topic : console_topics) {
        fmt::print("{} ", topic);
    }
    fmt::print("\n");
    fmt::print("[TWSB Coms][INFO] Press 'exit' to quit\n");

    while (true) {
        FD_ZERO(&read_fds);
        FD_SET(serial_fd, &read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
        FD_SET(subSocketFd, &read_fds);  

        int max_fd = std::max({serial_fd, STDIN_FILENO, subSocketFd});
        if (select(max_fd + 1, &read_fds, nullptr, nullptr, nullptr) < 0) {
            std::cerr << "Error in select\n";
            break;
        }
        if (FD_ISSET(serial_fd, &read_fds)) {  // Handle incoming telemetry messages from the serial port
            grabMsgFrame(serial_fd, coms);  // decodes serial buffer inplace
            while (!coms.telemMsgQ.empty()) {
                TelemetryMsg msg = coms.telemMsgQ.front();
                coms.telemMsgQ.pop(); 
                publishTimeStampedMsg(pubSocket, coms, msg); // publish timestamped message
                if (console_topics.find(msg.topic) != console_topics.end()) {
                    displayMsgConsole(msg); // Debug console output
                }   
            }
        }
        if (FD_ISSET(STDIN_FILENO, &read_fds)) { // Handle user input
            std::string cmd;
            std::getline(std::cin, cmd);
            if (cmd == "exit"){fmt::print("[TWSB_COMS] Exiting"); break;} 
            std::string packed_cmd = packCommand(cmd, coms.serial_protocol.sof_byte, coms.serial_protocol.eof_byte);
            write(serial_fd, packed_cmd.c_str(), packed_cmd.size());
        }
        // Handle ZMQ Sub Msg
        if (FD_ISSET(subSocketFd, &read_fds)) {  // Handle ZMQ subscription messages
            zmq::message_t topic_msg;
            zmq::message_t data_msg;
            subSocket.recv(topic_msg, zmq::recv_flags::dontwait);
            subSocket.recv(data_msg, zmq::recv_flags::dontwait);


            std::string command(static_cast<char*>(data_msg.data()), data_msg.size());
            fmt::print("[TWSB_COMS] Received Command: {}\n", command);

            std::string packed_cmd = packCommand(command, coms.serial_protocol.sof_byte, coms.serial_protocol.eof_byte);
            write(serial_fd, packed_cmd.c_str(), packed_cmd.size());
        }
        
    }

    close(serial_fd);
    return 0;
}