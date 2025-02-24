
/******************************************************
 * @file remoteControlServer.cpp
 * @brief Remote control server for the TWSB
 * @detail Listens for messages from remote controller on udp port 5557
 *        and forwards the messages to the TWSB over ZMQ
 *        The messages are formatted as per the pub-rpc protocol
 */


#include <iostream>
#include <cstring>
#include <stdexcept>
#include <zmq.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "../common/coms.hpp"
#include "../common/common.hpp"

#define UDP_PORT 5557      
#define BUFFER_SIZE 512  
  


#define NODE_NAME "RCTRL"

int main() {
    int udp_sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(udp_sock_ == -1) {
        throw std::runtime_error("UDP socket creation failed: " + std::string(strerror(errno)));
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    if(bind(udp_sock_, (sockaddr*)&server_addr, sizeof(server_addr)) ){
        close(udp_sock_);
        throw std::runtime_error("UDP bind failed: " +  std::string(strerror(errno)));
    }
    zmq::context_t zmq_context(1);
    zmq::socket_t zmq_sock(zmq_context, zmq::socket_type::pub);
    zmq_sock.set(zmq::sockopt::linger, 0);
    zmq_sock.connect(CMD_SRC_SOCKET);

    char buffer[BUFFER_SIZE];
    while(true) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        ssize_t msg_len = recvfrom(udp_sock_, buffer, BUFFER_SIZE, 0, (sockaddr*)&client_addr, &addr_len);//blocking

        if(msg_len == -1) { 
            std::cerr << "Error receiving UDP message: " << strerror(errno) << std::endl;
            continue;
        }

        zmq::message_t address_msg("TWSB", 4);
        zmq::message_t cmd_msg(buffer, msg_len);
        try {
            zmq_sock.send(address_msg, zmq::send_flags::sndmore);
            zmq_sock.send(cmd_msg, zmq::send_flags::none);
        } catch(const zmq::error_t& e) {
            std::cerr << "ZMQ send error: " << e.what() << std::endl;
        }
        if(DEBUG_MODE){
            std::cout << "Sent ZMQ message: " << buffer << std::endl;
        }
    }
    return 0;
}