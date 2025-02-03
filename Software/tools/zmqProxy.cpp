/************************************************************************************
 *  @file    zmqProxy.cpp
 *  @brief   ZeroMQ Proxy implementation with proper cleanup
 *  @date    2025-01-09
 *  @version 1.1.0
 *
 *  ZeroMQ Proxy CLI tool.
 *  Up to 5 input sockets are supported, which are proxied through to the output in a round-robin fashion.
 *  Handles SIGINT (Ctrl+C) gracefully.
 *  Distributed under MIT license.
 ************************************************************************************/

#include <zmq.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <unistd.h>

std::atomic<bool> running(true);

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nSIGINT received. Shutting down gracefully...\n";
        running = false;
    }
}

void printProxyInfo(const std::vector<std::string>& input_sckAddrs, const std::string& output_Addr) {
    std::cout << "Proxy started.\n";
    std::cout << "\033[1;34m" << "Proxy Info" << "\033[0m" << std::endl;
    std::cout << "\033[1;36m" << "+----------------------+----------------------+" << "\033[0m" << std::endl;
    std::cout << "\033[1;36m" << "| Input Address        | Output Address       |" << "\033[0m" << std::endl;
    std::cout << "\033[1;36m" << "+----------------------+----------------------+" << "\033[0m" << std::endl;
    
    for (const auto& addr : input_sckAddrs) {
        std::cout << "| " << std::left << std::setw(20) << addr << " | " << std::setw(20) << output_Addr << " |" << std::endl;
    }
    
    std::cout << "\033[1;36m" << "+----------------------+----------------------+" << "\033[0m" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 7) {  // 1 output + 1 to 5 input sockets
        std::cerr << "Usage: " << argv[0] << " <output_address> <input_address1> [input_address2] [input_address3] [input_address4] [input_address5]\n";
        return 1;
    }

    std::string output_Addr = argv[1];
    std::vector<std::string> input_sckAddrs;

    for (int i = 2; i < argc; ++i) {
        input_sckAddrs.push_back(argv[i]);
    }

    // Register SIGINT handler
    signal(SIGINT, signalHandler);

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.set(zmq::sockopt::linger, 0);
    subscriber.set(zmq::sockopt::subscribe, "");
    subscriber.set(zmq::sockopt::ipv6, 1);

    for (const auto& sckAddr : input_sckAddrs) {
        try {
            subscriber.connect(sckAddr);
        } catch (const zmq::error_t& e) {
            std::cerr << "Error connecting to input socket: " << e.what() << "\n";
            return 1;
        }
    }

    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.set(zmq::sockopt::linger, 0);
    publisher.set(zmq::sockopt::ipv6, 1);

    try {
        publisher.bind(output_Addr);
    } catch (const zmq::error_t& e) {
        std::cerr << "Error binding output socket: " << e.what() << "\n";
        return 1;
    }

    printProxyInfo(input_sckAddrs, output_Addr);

    // Proxy message loop
    try {
        while (running) {
            zmq::proxy(subscriber, publisher);
        }
    } catch (const zmq::error_t& e) {
        if (running) {
            std::cerr << "Proxy error: " << e.what() << "\n";
        }
    }

    // Cleanup
    std::cout << "Cleaning up resources...\n";
    subscriber.close();
    publisher.close();
    context.close();

    std::cout << "Proxy stopped.\n";
    return 0;
}
