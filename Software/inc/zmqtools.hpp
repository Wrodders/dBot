#ifndef ZMQTOOLS_HPP
#define ZMQTOOLS_HPP

#include <zmq.hpp>
#include "../common/common.hpp"


void printUsage(const std::string &programName) {
    std::cerr << "Usage: " << programName << " [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -h        Display this help message" << std::endl;
    std::cerr << "  -s SOCKET Set the ZMQ socket (default: tcp://localhost:5555)" << std::endl;
    std::cerr << "  -t TOPIC  Set the ZMQ topic (default: "")" << std::endl;
    std::cerr << "  -m MODE   Set the ZMQ socket mode (default: connect)" << std::endl;
}

void handleCLI(int argc, char *argv[], std::string &socket_address, std::string &topic_string, std::string &mode) {
    int opt;
    while ((opt = getopt(argc, argv, "hs:t:m:")) != -1) {
        switch (opt) {
            case 'h':
                printUsage(argv[0]);
                exit(0);
            case 's':
                socket_address = optarg;
                break;
            case 't':
                topic_string = optarg ? optarg : "";
                break;
            case 'm':
                mode = optarg;
                break;

            default:
                printUsage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }
}

#endif // ZMQTOOLS_HPP  