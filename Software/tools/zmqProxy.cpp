/************************************************************************************
 *  @file    zmqProxy.cpp
 *  @brief   ZeroMQ Proxy implementation
 *  @date    2025-01-09
 *  @version 1.0.0
 *
 *  ZeroMQ Proxy CLI tool.
 *  Up to 5 input sockets are supported which are proxied through to the output in a round-robin fashion.
 *  Distributed under MIT license
 ************************************************************************************/  

#include <zmq.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <unistd.h>

void printProxyInfo(std::vector<std::string> input_sckAddrs, std::string output_Addr) {
    std::cout << "Proxy started.\n";
    std::cout << "+-------------------+-------------------------+\n";
    std::cout << "|     Socket Type   |         Address         |\n";
    std::cout << "+-------------------+-------------------------+\n";
    for (const auto& input_socket : input_sckAddrs) {
        std::cout << "| Input Socket      | " << input_socket << " |\n";
    }
    std::cout << "| Output Socket     | " << output_Addr << " |\n";
    std::cout << "+-------------------+-------------------+\n";
}

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 7) {  // 1 output + 1 to 5 input sockets
        std::cerr << "Usage: " << argv[0] << " <output_address> <input_address1> [input_address2] [input_address3] [input_address4] [input_address5] \n";
        return 1;
    }

    std::string output_Addr = argv[1];
    std::vector<std::string> input_sckAddrs;

    // Collect input sockets
    for (int i = 2; i < argc; ++i) {
        input_sckAddrs.push_back(argv[i]);
    }

    
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    for (const auto& sckAddr : input_sckAddrs)
        try
        {
            subscriber.connect(sckAddr);
        }
        catch (const zmq::error_t& e)
        {
            std::cerr << "Error connecting to input socket: " << e.what() << "\n";
            return 1;
        }
        
    subscriber.set(zmq::sockopt::subscribe, "");
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(output_Addr);

    // Call the function to print proxy info
    printProxyInfo(input_sckAddrs, output_Addr);
    std::atomic<bool> running(true);

    try {
        zmq::proxy(subscriber, publisher);
    } catch (const zmq::error_t& e) {
        std::cerr << "Proxy error: " << e.what() << "\n";
    }

    running = false;
    std::cout << "Proxy stopped.\n";
    return 0;
}
