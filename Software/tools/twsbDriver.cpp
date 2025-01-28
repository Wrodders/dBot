#include  "../inc/twsbDriver.hpp"
// ********* CLI UTILS ************************************************************************* //
// Display usage message for the program
void printUsage(const std::string &programName) {
    std::cerr << "Usage: " << programName << " [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "-h        Display this help message" << std::endl;
    std::cerr << "-p PORT   Set the serial port (default: /dev/ttyUSB0)" << std::endl;
    std::cerr << "-b BAUD   Set the baud rate (default: 9600)" << std::endl;
    std::cerr << "-s SOCKET Set the ZMQ socket (default: tcp://localhost:5555)" << std::endl;
}

bool getCLI(int argc, char* argv[], std::string &port, int &baud) {
    int opt;
    while ((opt = getopt(argc, argv, "hp:b:")) != -1) {
        switch (opt) {
            case 'h':
                printUsage(argv[0]);
                return false;
            case 'p':
                port = optarg;
                break;
            case 'b':
                baud = atoi(optarg);
                break;
            default:
                printUsage(argv[0]);
                return false;
        }
    }
    return true;
}
//@Brief: Read data from the serial port and update the telemetry data
int handleSerialPortRead(TWSBDriver &twsbDriver) {
    twsbDriver.update();
    return 0;
}
//@Brief: Publish the TwoWheelSelfBalancing telemetry data over ZMQ
int publishTWSB(zmq::socket_t &pubSocket, TWSBDriver &twsbDriver) {
    std::string msg;
    //Serialize the telemetry data with the protocol
    for (int i = 0; i < T_NUM_VARS; i++) {
        msg += std::to_string(twsbDriver.telemetry_.stateVars[i]);
        msg += twsbDriver.protocol.delim_byte;
    }
    // serialize the timestamp
   
    // publish under SERIAL topic
    std::string topic = "SERIAL/STATE/";
    for(int i = 0; i < T_NUM_VARS; i++){
        //get pubslish string 
        topic += stateVarString((enum StateVars)i);
        //send the topic and message
        pubSocket.send(zmq::message_t(topic), zmq::send_flags::sndmore);
        pubSocket.send(zmq::message_t(twsbDriver.getTelemetryValue((enum StateVars)i)), zmq::send_flags::sndmore);
    }
  
    if(twsbDriver.cmdret_.msg != "" ){
        // Publish the command return message
        zmq::message_t topic("SERIAL/CMDRET/", 14);
        zmq::message_t message(twsbDriver.cmdret_.msg.c_str(), twsbDriver.cmdret_.msg.size());
        pubSocket.send(topic, zmq::send_flags::sndmore);
        pubSocket.send(message, zmq::send_flags::none);
    }
    if(twsbDriver.logentry_.message != "" ){
        // Publish the log entry message
        zmq::message_t topic("SERIAL/LOG/", 11);
        zmq::message_t message(twsbDriver.logentry_.message.c_str(), twsbDriver.logentry_.message.size());
        pubSocket.send(topic, zmq::send_flags::sndmore);
        pubSocket.send(message, zmq::send_flags::none);
    }


    return 0;
}

int handleZMQCmd(zmq::socket_t &subSocket, TWSBDriver &twsbDriver) {
    zmq::message_t msg;
    zmq::message_t topic; // Topic message recevive more
    subSocket.recv(topic, zmq::recv_flags::dontwait);
    subSocket.recv(msg, zmq::recv_flags::dontwait);

    std::string topicStr = std::string(static_cast<char*>(topic.data()), topic.size());
    std::string msgStr = std::string(static_cast<char*>(msg.data()), msg.size());
    return 0;
}


int main(int argc, char* argv[]) {
    // Default port and baud rate
    std::string port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;
    if (!getCLI(argc, argv, port, baud)) {
        return -1;
    }
    // Initialize ZeroMQ 
    zmq::context_t context;
    zmq::socket_t subSocket = zmq::socket_t(context, zmq::socket_type::sub);
    subSocket.connect("ipc:///tmp/botCMDS"); // General Bot Commands Socket
    subSocket.set(zmq::sockopt::subscribe, "SERIAL"); // Receive commands for the serial port
    zmq::socket_t pubSocket = zmq::socket_t(context, zmq::socket_type::pub);
    pubSocket.bind("ipc:///tmp/botMSGS"); // General Bot Messages Socket 
    // Create a TWSBDriver object
    TWSBDriver twsbDriver(port, baud);
    int zmqSubFd = subSocket.get(zmq::sockopt::fd);
    int zmqPubFd = pubSocket.get(zmq::sockopt::fd);
    while (true) {
        // Prepare the file descriptor set for select
        fd_set readFds;
        FD_ZERO(&readFds);
        FD_SET(twsbDriver.serialFd, &readFds);
        FD_SET(zmqSubFd, &readFds); // Add the ZMQ socket to the file descriptor set
        FD_SET(zmqPubFd, &readFds); // Add the ZMQ socket to the file descriptor set
        int maxFd = twsbDriver.serialFd; // Initialize the maximum file descriptor
        if (zmqSubFd > maxFd) maxFd = zmqSubFd; // Update the maximum file descriptor
        int ready = select(maxFd + 1, &readFds, nullptr, nullptr, nullptr);
        if (ready == -1) {
            std::cerr << "Error: select() failed" << std::endl;
            break;
        }
        if (FD_ISSET(twsbDriver.serialFd, &readFds)) { // Serial port data available
            handleSerialPortRead(twsbDriver);
            if (!DEBUG_MODE){twsbDriver.printTelemetryTable();}
            publishTWSB(pubSocket, twsbDriver); // 
        }
        if (FD_ISSET(zmqSubFd, &readFds)) { // ZMQ socket data available
            handleZMQCmd(subSocket, twsbDriver);
        }
    }

    // cleanup
    close(zmqSubFd);
    close(zmqPubFd);
    return 0;
}