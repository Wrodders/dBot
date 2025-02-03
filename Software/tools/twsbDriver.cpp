#include  "../inc/twsbDriver.hpp"
/* Two Wheel Self Balancing Driver ---------------------------------------------- /

    * Publishes telemetry data over ZMQ IPC
    * Writes Logs to syslog
    * Publishes command return messages over ZMQ IPC

*/






// ********* CLI UTILS ************************************************************************* //
// Display usage message for the program
void printUsage(const std::string &programName) {
    std::cerr << "Usage: " << programName << " [options]\n";
    std::cerr << "Options:\n";
    std::cerr << "-h        Display this help message\n" ;
    std::cerr << "-p PORT   Set the serial port (default: /dev/ttyUSB0)\n" ;
    std::cerr << "-b BAUD   Set the baud rate (default: 9600)\n";
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



// ********* MAIN ************************************************************************* //

int main(int argc, char* argv[]) {
    std::string port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;
    if (!getCLI(argc, argv, port, baud)) {
        return -1;
    }

    zmq::context_t context;
    
    // **ZMQ Subscriber (for commands)**
    zmq::socket_t subSocket(context, zmq::socket_type::sub);
    subSocket.set(zmq::sockopt::linger, 0);
    subSocket.connect("ipc:///tmp/botCMDS");  // FIXED: Use connect() instead of bind()
    subSocket.set(zmq::sockopt::subscribe, "TWSB"); 

    // **ZMQ Publisher (for telemetry)**
    zmq::socket_t pubSocket(context, zmq::socket_type::pub);
    pubSocket.bind("ipc:///tmp/botMSGS");

    // **Main Loop**
    while(true){}


}
