/*************************************************
 *  @file    zmqsub.cpp
 *  @brief   zmq topic subscriber CLI tool   
 *  @date    2025-01-10
 *  @version 1.0.0
 * CLI tool connects to an endpoint and subscribes to a topic
*/

#include <zmq.hpp>
#include <iostream>
#include <getopt.h>
#include <csignal>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <fmt/core.h>

#include "../common/common.hpp"

#define NODE_NAME "ZMQSUB"

void printUsage(const std::string &programName) {
    std::cerr << "Usage: " << programName << " [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  -h        Display this help message" << std::endl;
    std::cerr << "  -s SOCKET Set the ZMQ socket (default: tcp://localhost:5555)" << std::endl;
    std::cerr << "  -t TOPIC  Set the ZMQ topic (default: "")" << std::endl;
}

void handleCLI(int argc, char *argv[], std::string &socket_address, std::string &topic_string) {
    int opt;
    while ((opt = getopt(argc, argv, "hs:t:")) != -1) {
        switch (opt) {
            case 'h':
                printUsage(argv[0]);
                exit(0);
            case 's':
                socket_address = optarg;
                break;
            case 't':
                topic_string = optarg;
                break;
            default:
                printUsage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }
}

int main(int argc, char *argv[]) {
    std::string socket_address = "tcp://localhost:5555";
    std::string topic_string = "";
    handleCLI(argc, argv, socket_address, topic_string);

    zmq::context_t context(1);
    zmq::socket_t subSocket(context, zmq::socket_type::sub);
    subSocket.set(zmq::sockopt::linger, 0);
    subSocket.set(zmq::sockopt::subscribe, topic_string.c_str());
    subSocket.connect(socket_address);

    fmt::print("[ZMQ Subscriber] Connected to {}\n", socket_address);
    fmt::print("[ZMQ Subscriber] Subscribed to {}\n", topic_string);

    while (true) {
        zmq::message_t msg;
        (void) subSocket.recv(msg, zmq::recv_flags::none);
        zmq::recv_result_t recv = subSocket.recv(msg, zmq::recv_flags::none);
        // print the message
        fmt::print("[{}] {}\n", topic_string, std::string(static_cast<char*>(msg.data()), msg.size()));
    }
   fmt::print("[{}] Exiting\n", NODE_NAME);
    return 0;
}
