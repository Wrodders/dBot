/************************************************************************************
 *  @file    twsbComs.cpp
 *  @brief   Implementation of pub-rpc protocol for the TWSB over ZMQ
 *  @date    2025-01-05
 *  @version 1.1.0
 *  Single-threaded Event Driven IO loop
 *  Parses in-place the xMacro string mapped to the decoded Publisher ID
 *  Bridges commands send over the ZMQ IPC socket to the TWSB over UART
 *  Provides raw command and topic subscription console to the serial port for debugging
 *  Publishes Timestamped Serialized Telemetry Messages to the ZMQ IPC socket
 * 
 *          ----> | ZMQ SUB | ----> | TWSB UART WRITE | ----> | TWSB |
 *          ----> | Console | ----> | TWSB UART WRITE | ----> | TWSB |
 *          <---- | ZMQ PUB | <---- | TWSB UART READ  | <---- | TWSB |
 ************************************************************************************/


#include <unistd.h>
#include <termios.h>


#include "../common/common.hpp"
#include "../common/coms.hpp"


//*** TWSB Coms Console */
#define NODE_NAME "TWSB"

#define DEFAULT_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUD 115200

// ********* CLI UTILS ************************************************************************* //
void cli_display_help(const std::string& program_name) {
    fmt::print("Usage: {} <serial_port> <baud_rate> <topic1> <topic2> ...\n", program_name);
    fmt::print("Example: {} /dev/ttyUSB0 115200 IMU STATE\n", program_name);
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


// ********* MAIN ************************************************************************* //
int main(int argc, char* argv[]) {
    // --------------- CLI Parsing ----------------- //
    if (argc < 4) {
        cli_display_help(argv[0]);
        return 1;
    }

    std::string port = argv[1];
    int baud_rate = std::stoi(argv[2]);

    // Store console topics in a set
    std::unordered_set<std::string> console_topics; // group of topics to display on the console
    for (int i = 3; i < argc; i++) {
        console_topics.insert(argv[i]);
    }

    // ---------- Communication Setup -------------- //
    Protocol proto = {'<', '\n', ':', 'A'};
    Command cmd; // Working command variable
    NodeConfigManager config("robotConfig.json", NODE_NAME); // Load the configuration file 
    struct SerComs coms; // Initialize the SerComs struct
    /* Note this Node is a transparent bridge between the ZMQ and the TWSB
    // All Parameters and Publishers are of the TWSB MCU as implemented inthe Firmware
    // The Config is just loaded to provide a means of maping the publisher id to the
    // string name as required by the ZMQ IPC message */
    
    // --------------- IO Setup ----------------- //
    int serial_fd = config_serial_port(port, baud_rate);
    if (serial_fd == -1) return 1;
    // Make console input non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); 

    zmq::context_t context(1);
    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, NODE_NAME);
    cmd_subsock.connect(CMD_SOCKET_ADDRESS); // Connect to the command socket proxy
    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.bind(MSG_PUB_ADDRESS); // Bind to the telemetry socket proxy
    // --------------- Console Setup ----------------- //
    fmt::print("[TWSB Coms][INFO] Begin TWSB Console\n");
    fmt::print("[TWSB Coms][INFO] Console Topic Filter: ");
    for (const auto& topic : console_topics) {
        fmt::print("{} ", topic);
    }
    fmt::print("\n");
    fmt::print("[TWSB Coms][INFO] Press 'exit' to quit\n");
    // ------------- Event Setup ----------------- //
    zmq::pollitem_t poll_items[] = {
        { nullptr, serial_fd, ZMQ_POLLIN, 0 }, // Serial input
        { nullptr, STDIN_FILENO, ZMQ_POLLIN, 0 }, // Console input
        { static_cast<void*>(cmd_subsock), 0, ZMQ_POLLIN, 0 } // ZeroMQ subscription
    };
    // -------------- Event Loop ----------------- //
    while (true) {
        int rc = zmq::poll(poll_items, 3, std::chrono::milliseconds(-1));
        if (rc == -1) {
            fmt::print("[TWSB_COMS] Error: Polling Error\n");
            break;
        } 
        /* Serial Telemetry Bridge
        @brief: Publishes Timestamped, ascii serialized messages from the serial port
                under string topic names over ZMQ IPC. 
                Also displays select messages on te debug console
        */
        if (poll_items[0].revents & ZMQ_POLLIN) { // SerialPort
            sercoms_grab_telemetry(serial_fd, coms, proto, config);  // read and half parse messages from the serial port
            while (!coms.telemMsgQ.empty()) { // Publish all messages in the queue
                struct TelemetryMsg telem_msg = coms.telemMsgQ.front(); // grab the message from the queue
                coms.telemMsgQ.pop(); 
                zmqcoms_publish_tsmp_msg(msg_pubsock,telem_msg); // bridge the message to the ZMQ
                if (console_topics.find(telem_msg.topic) != console_topics.end()) {
                    display_console(telem_msg.topic, telem_msg.data, formatTimestamp(telem_msg.timestamp));
                }
            }
        }
        if (poll_items[1].revents & ZMQ_POLLIN) { // Console Input
            std::string cmd_input;
            std::getline(std::cin, cmd_input);
            if (cmd_input.empty()) { return 0; }
            cmd_input = proto_pack_asciicmd(cmd_input, proto);
            fmt::print("\033[2J\033[1;1H"); // Clear the screen
            if (cmd_input == "exit") {
                fmt::print("[TWSB_COMS] Exiting\n");
                break;
            }
            // Send the command to the TWSB 
            write(serial_fd, cmd_input.c_str(), cmd_input.size());
        }

        if (poll_items[2].revents & ZMQ_POLLIN) { // Command Message Available 
            std::tuple<std::string, std::string> cmd_msg_packet = zmqcoms_receive_asciicmd(cmd_subsock);
            std::string cmd_msg = std::get<1>(cmd_msg_packet);
            display_console(std::get<0>(cmd_msg_packet), cmd_msg, formatTimestamp(std::chrono::system_clock::now()));
            if (cmd_msg == "exit") {
                fmt::print("[TWSB_COMS] Exiting\n");
                break;
            }
            // Send the command msg to the TWSB transparently
            write(serial_fd, cmd_msg.c_str(), cmd_msg.size());
           
        }
    }

    close(serial_fd);
    return 0;
}



