/*************************************************
 * @file zmqPubTest.cpp
 * @brief zmq topic publisher CLI tool 
 * @details Publishes line terminated messages from stdin to a ZMQ socket under a specified topic
 */

#include "../inc/zmqtools.hpp"

#define NODE_NAME "ZMQPUB"

int main(int argc, char *argv[]) {
    std::string socket_address = "tcp://localhost:5555";
    std::string topic_string = "";
    std::string mode = "connect";
    handleCLI(argc, argv, socket_address, topic_string, mode);

    zmq::context_t context(1);
    zmq::socket_t pubSocket(context, zmq::socket_type::pub);
    pubSocket.set(zmq::sockopt::linger, 0);
    if(mode == "bind"){
        pubSocket.bind(socket_address);
    }else{
        pubSocket.connect(socket_address);
    }
    std::cout << "[" << NODE_NAME << "] " << socket_address << " to " << mode << std::endl;
    std::cout << "[" << NODE_NAME << "] Publishing under topic: " << topic_string << std::endl;
    std::cout << "[" << NODE_NAME << "] Enter messages to publish under topic: " << topic_string << std::endl;
    while (true) {
        // Build topic
        zmq::message_t topic(topic_string.size()); 
        memcpy(topic.data(), topic_string.c_str(), topic_string.size());
        // Grab the message from stdin
        std::string msg_str;
        std::getline(std::cin, msg_str);
        // Build the message
        msg_str.push_back('\n');
        zmq::message_t msg(msg_str.size());
        memcpy(msg.data(), msg_str.c_str(), msg_str.size());
        // Send the topic and message
        pubSocket.send(topic, zmq::send_flags::sndmore);
        pubSocket.send(msg, zmq::send_flags::none); 
    }
    // Clean up
    std::cout << "[" << NODE_NAME << "] Exiting..." << std::endl;
    pubSocket.close();
    return 0;
}

