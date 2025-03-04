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
 
#include "../inc/twsb.hpp"

// ********* MAIN ************************************************************************* //
int main(int argc, char *argv[]) {
    // --------------- CLI Parsing ----------------- //
    if (argc < 3) {
        syslog(LOG_ERR, "Usage: %s <serial_port> <baud_rate>\n", argv[0]);
        return 1;
    }
    std::string port = argv[1];
    int baud_rate = std::stoi(argv[2]);
    if (port.empty() || baud_rate == 0) {
        syslog(LOG_ERR, "Error: Invalid Serial Port or Baud Rate\n");
        return 1;
    }

    // ---------- Communication Setup -------------- //
    Protocol proto = {.sof_byte = '<', .eof_byte = '\n', .delim_byte = ':', .offset = 'A'};
    CommandMsg cmd; // Working command variable
    PubMap pub_map("/home/pi/prog/software/configs/robotConfig.json", NODE_NAME); 
    struct SerComs coms;
    /* Note this Node is a transparent bridge between the ZMQ and the TWSB
    // All Parameters and Publishers are of the TWSB MCU as implemented in the Firmware
    // The Config is just loaded to provide a means of maping the publisher id to the
    // sting name as required by the ZMQ IPC message */
    
    // --------------- IO Setup ----------------- //
    syslog(LOG_INFO, "Configuring serial port %s at %d baud", port.c_str(), baud_rate);
    int serial_fd = config_serial_port(port, baud_rate);
    if (serial_fd == -1) {
        syslog(LOG_CRIT, "Error: Unable to open serial port %s\n", port.c_str());
        return 1;
    }
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0); // Make console input non-blocking
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); 
    // --------------- ZMQ Setup ----------------- //
    zmq::context_t context(1);
    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, NODE_NAME);
    cmd_subsock.bind("tcp://*:5556"); // remote command server
    cmd_subsock.bind("ipc:///tmp/botcmds"); // local command server
    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.bind("tcp://*:5555");  // message server    
    // --------------- Console Setup ----------------- //
   
    zmq::pollitem_t poll_items[] = {
        { nullptr, serial_fd, ZMQ_POLLIN, 0 }, 
        { static_cast<void*>(cmd_subsock), 0, ZMQ_POLLIN, 0 } 
    };
    // -------------- Event Loop ----------------- //
    syslog(LOG_INFO, "Starting Event Loop");
    int rst_attempts = 0;
    while (true) {
        int rc = zmq::poll(poll_items, 2, std::chrono::milliseconds(30000)); // 30 sec timeout
        if (rc == -1) {
            syslog(LOG_CRIT, "Error: Polling Error\n");
            break;
        } 
        if (rc == 0) { // Polling Timeout Attempt to Reset the MCU
            syslog(LOG_ERR, "Error: Polling Timeout\n");
            rst_attempts++;
            syslog(LOG_WARNING, "Resetting TWSB MCU Attempt: %d", rst_attempts);
            system("bash -c gpioset gpiochip0 2=0");
            usleep(100000);
            system("bash -c gpioget gpiochip0 2");
            if (rst_attempts >= 3) {
                syslog(LOG_CRIT, "Error: Reset Attempts Exceeded\n");
                return 1;
            }
        }
        // --------------- Event Handling ----------------- //
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
            // Send the command msg to the TWSB MCU transparently
            write(serial_fd, cmd.data.c_str(), cmd.data.size());
        }
    }

    close(serial_fd);
    return 0;
}