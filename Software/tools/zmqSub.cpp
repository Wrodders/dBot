/*************************************************
 *  @file    zmqsub.cpp
 *  @brief   zmq topic subscriber CLI tool   
 *  @date    2025-01-10
 *  @version 1.0.0
 * CLI tool connects to an endpoint and subscribes to a topic
*/

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
    std::cout << "[ZMQ Subscriber] Socket " << mode << "    " << socket_address << std::endl;
    std::cout << "[ZMQ Subscriber] Subscribed to topic: " << topic_string << std::endl;


    while (true) {
        zmq::message_t msg;
        (void) subSocket.recv(msg, zmq::recv_flags::none);
        std::cout << "[" << NODE_NAME << "] " << std::string(static_cast<char*>(msg.data()), msg.size()) << std::endl;
    }
    // Clean up
    std::cout << "[" << NODE_NAME << "] Exiting..." << std::endl;
    subSocket.close();
    return 0;
}
