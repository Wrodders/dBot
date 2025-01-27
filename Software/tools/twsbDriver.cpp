#include  "../inc/twsbDriver.hpp"


// ********* CLI UTILS ************************************************************************* //
// Display usage message for the program
void printUsage(const std::string &programName) {
    std::cerr << "Usage: " << programName << " [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "-h        Display this help message" << std::endl;
    std::cerr << "-p PORT   Set the serial port (default: /dev/ttyUSB0)" << std::endl;
    std::cerr << "-b BAUD   Set the baud rate (default: 9600)" << std::endl;
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

int main(int argc, char* argv[]) {
    // Default port and baud rate
    std::string port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD;
    if(!getCLI(argc, argv, port, baud)) {
        return -1;
    }
    zmq::context_t context;
    zmq::socket_t subSocket = zmq::socket_t(context, zmq::socket_type::sub);
    subSocket.connect("tcp://localhost:5555");
    subSocket.set(zmq::sockopt::subscribe, "");
    // Create a TWSBDriver object
    TWSBDriver twsbDriver(port, baud);
    

    while (true) {
        fd_set readFds;
        FD_ZERO(&readFds);

        FD_SET(twsbDriver.serialFd, &readFds);
        int maxFd = twsbDriver.serialFd;

        int zmqFd = subSocket.get(zmq::sockopt::fd);
        FD_SET(zmqFd, &readFds);
        if (zmqFd > maxFd) maxFd = zmqFd;

        int ready = select(maxFd + 1, &readFds, nullptr, nullptr, nullptr);
        if (ready == -1) {
            std::cerr << "Error: select() failed" << std::endl;
            break;
        }

        if (FD_ISSET(twsbDriver.serialFd, &readFds)) {
            twsbDriver.update();
            if(DEBUG_MODE==false){twsbDriver.printTelemetryTable();}

        }

        if (FD_ISSET(zmqFd, &readFds)) {
            continue;
        }

    }



  

    return 0;
}