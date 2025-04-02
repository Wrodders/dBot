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

#include "../inc/zmqtools.hpp"

#define NODE_NAME "ZMQSUB"

int main(int argc, char *argv[]) {
    std::string socket_address = "tcp://localhost:5555"; // defaults 
    std::string topic_string = "";
    std::string mode = "connect";
    handleCLI(argc, argv, socket_address, topic_string, mode);

    zmq::context_t context(1);
    zmq::socket_t subSocket(context, zmq::socket_type::sub);
    subSocket.set(zmq::sockopt::linger, 0);
    subSocket.set(zmq::sockopt::subscribe, topic_string.c_str());
    if(mode == "bind"){
        subSocket.bind(socket_address);
    }else{
        subSocket.connect(socket_address);
    }

    fmt::print("[ZMQ Subscriber] {} to {}\n",mode, socket_address);
    fmt::print("[ZMQ Subscriber] Subscribed to {}\n", topic_string);

    while (true) {
        zmq::message_t msg;
        (void) subSocket.recv(msg, zmq::recv_flags::none);
        fmt::print("[{}] {}\n", NODE_NAME, std::string(static_cast<char*>(msg.data()), msg.size()));
    }
   fmt::print("[{}] Exiting\n", NODE_NAME);
    return 0;
}
