/*************************************************
 * @file zmqPubTest.cpp
 * @brief zmq topic publisher CLI tool 
 * @details Publishes line terminated messages from stdin to a ZMQ socket under a specified topic
 */

#include <zmq.hpp>
#include <iostream>
#include <getopt.h>
#include <csignal>
#include <iomanip>
#include <atomic>
#include <fmt/core.h>

#define NODE_NAME "ZMQPUB"

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
    zmq::socket_t pubSocket(context, zmq::socket_type::pub);
    pubSocket.set(zmq::sockopt::linger, 0);
    pubSocket.bind(socket_address);
    fmt::print("[{}] Bound to {}\n",NODE_NAME, socket_address);
    fmt::print("[{}] Publishing under topic: {}\n",NODE_NAME, topic_string);
    fmt::print("[{}] Enter messages to publish under topic: {}\n",NODE_NAME, topic_string);
    while (true) {
        // Build topic
        zmq::message_t topic(topic_string.size()); 
        memcpy(topic.data(), topic_string.c_str(), topic_string.size());
        // Grab the message from stdin
        std::string msg_str;
        std::getline(std::cin, msg_str);
        // Build the message
        zmq::message_t msg(msg_str.size());
        memcpy(msg.data(), msg_str.c_str(), msg_str.size());
        // Send the topic and message
        pubSocket.send(topic, zmq::send_flags::sndmore);
        pubSocket.send(msg, zmq::send_flags::none); 
    }

    
    return 0;
}
