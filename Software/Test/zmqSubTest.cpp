#include <zmq.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
    zmq::context_t context;
    zmq::socket_t subSocket = zmq::socket_t(context, zmq::socket_type::sub);
    subSocket.set(zmq::sockopt::subscribe, "");
    subSocket.connect("ipc:///tmp/botMSGS"); // General Bot Messages Socket 
    while(true){
        zmq::message_t topic, msg;
        subSocket.recv(topic, zmq::recv_flags::none);
        subSocket.recv(msg, zmq::recv_flags::none);
        std::string message = std::string(static_cast<char*>(msg.data()), msg.size());
        std::cout << "Received: " << message << std::endl;
    }
    return 0;
}

