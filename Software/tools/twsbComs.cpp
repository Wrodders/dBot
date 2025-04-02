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

    // --------------- ZMQ Setup ----------------- //
    zmq::context_t context(1);
    // External Interface
    zmq::socket_t excmd_subsock(context, zmq::socket_type::sub); 
    excmd_subsock.set(zmq::sockopt::linger, 0);
    excmd_subsock.set(zmq::sockopt::subscribe, "");
    excmd_subsock.bind("tcp://*:5556"); // remote command server
    zmq::socket_t exmsg_pubsock(context, zmq::socket_type::pub);
    exmsg_pubsock.set(zmq::sockopt::linger, 0);
    exmsg_pubsock.bind("tcp://*:5555");  // message server   
    // Internal Interface to robot services  acts as a proxy to the external interface
    zmq::socket_t botcmd_pubsock(context, zmq::socket_type::pub);
    botcmd_pubsock.set(zmq::sockopt::linger, 0);
    botcmd_pubsock.bind("ipc:///tmp/botcmds"); // local command server 
    zmq::socket_t botmsg_subsock(context, zmq::socket_type::sub);
    botmsg_subsock.set(zmq::sockopt::linger, 0);
    botmsg_subsock.connect("ipc:///tmp/botmsgs");  // local telemetry server
    botmsg_subsock.set(zmq::sockopt::subscribe, ""); 
    // Ugly Arch but it works to patch cmds form other services round robin
    zmq::socket_t mcucmd_subsock(context, zmq::socket_type::sub); // 
    mcucmd_subsock.set(zmq::sockopt::linger, 0);
    mcucmd_subsock.connect("ipc:///tmp/vizcmds");
    mcucmd_subsock.connect("ipc:///tmp/joycmds");
    mcucmd_subsock.set(zmq::sockopt::subscribe, "TWSB"); // Subscribe to all topics

    // --------------- Console Setup ----------------- //
    zmq::pollitem_t poll_items[] = {
        { nullptr, serial_fd, ZMQ_POLLIN, 0 },                  // Bridge Serial Port to ZMQ
        {static_cast<void*>(excmd_subsock), 0, ZMQ_POLLIN, 0 }, // TWSB Receives Commands from PC 
        {static_cast<void*>(botmsg_subsock), 0, ZMQ_POLLIN, 0}, // Forward Telemetry to PC
        {static_cast<void*>(mcucmd_subsock), 0, ZMQ_POLLIN, 0}  // Forward Commands to MCU

    };
    // -------------- Event Loop ----------------- //
    syslog(LOG_INFO, "Starting Event Loop");
    int rst_attempts = 0;
    try{
    while (true) {
        int rc = zmq::poll(poll_items, 4, std::chrono::milliseconds(30000)); // 30 sec timeout
        if (rc == -1) {
            syslog(LOG_CRIT, "Error: Polling Error\n");
            break;
        } 
        if (rc == 0) { // Polling Timeout Attempt to Reset the MCU
            syslog(LOG_ERR, "Error: Polling Timeout\n");
            rst_attempts++;
            syslog(LOG_WARNING, "Resetting TWSB MCU Attempt: %d", rst_attempts);
            system("bash -c gpioset 0 2=0"); // nRst = 0 and mcu enters reset state
            usleep(100000);
            system("bash -c gpioget 0 2"); // read -> input == high Z -> nRst = 1 and mcu exits reset
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
                coms_publish_tsmp_msg(exmsg_pubsock,telem_msg);
            }
        }
        if (poll_items[1].revents & ZMQ_POLLIN) { // External ZMQ Command Message Available 
            coms_receive_asciicmd(excmd_subsock, cmd); // Receive the command message
            if(cmd.topic == "TWSB"){  write(serial_fd, cmd.data.c_str(), cmd.data.size());}
            else{ // UGGGLY 
                syslog(LOG_INFO, "Forwarding Command to %s", cmd.topic.c_str());
                botcmd_pubsock.send(zmq::message_t(cmd.topic.c_str(), cmd.topic.size()), zmq::send_flags::sndmore);
                botcmd_pubsock.send(zmq::message_t(cmd.data.c_str(), cmd.data.size()), zmq::send_flags::none);
            }
        }
        // UCLYYYYYY Y AM I serdeserign messaged fromIPC to TCP ????? because deadlines and prev mistakes in arch
        if(poll_items[2].revents & ZMQ_POLLIN) { // Forward local telemetry to PC
            zmq::message_t topic, msg, timestamp;
            (void) botmsg_subsock.recv(topic, zmq::recv_flags::none);
            (void) botmsg_subsock.recv(msg, zmq::recv_flags::none);
            (void) botmsg_subsock.recv(timestamp, zmq::recv_flags::none);

            coms_publish_tsmp_msg(exmsg_pubsock, {std::string(static_cast<char*>(topic.data()), topic.size()),
                std::string(static_cast<char*>(msg.data()), msg.size()),
                std::string(static_cast<char*>(timestamp.data()), timestamp.size())});
        }

        if (poll_items[3].revents & ZMQ_POLLIN) {  // Received message for MCU locally
            coms_receive_asciicmd(mcucmd_subsock, cmd); // Receive the command message
            write(serial_fd, cmd.data.c_str(), cmd.data.size());
        }
    }
    }
    catch(const std::exception& e){
        syslog(LOG_ERR, "Error: %s", e.what());
    }

    close(serial_fd);
    return 0;
}