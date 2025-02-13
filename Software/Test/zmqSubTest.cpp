/*************************************************
 *  @file    zmqsub.cpp
 *  @brief   zmq topic subscriber CLI tool   
 *  @date    2025-01-10
 *  @version 1.0.0
 * CLI tool connects to an endpoint and subscribes to a topic
 * Formatted table using ASCII escape codes
 * Receives multipart messages Topic MsgData Timestamp 
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
        zmq::message_t topic, msg, timestamp;
        (void) subSocket.recv(topic, zmq::recv_flags::none);
        (void) subSocket.recv(msg, zmq::recv_flags::none);
        (void) subSocket.recv(timestamp, zmq::recv_flags::none);

        std::string topicStr(static_cast<char *>(topic.data()), topic.size());
        std::string msgStr(static_cast<char *>(msg.data()), msg.size());
        std::string timestampStr(static_cast<char *>(timestamp.data()), timestamp.size());
        // convert to formatted timestamp from chrono stead clock 
        std::chrono::time_point<std::chrono::system_clock> timestampPoint;
        timestampPoint = std::chrono::time_point<std::chrono::system_clock>(std::chrono::milliseconds(std::stoll(timestampStr)));
        std::time_t timestampTime = std::chrono::system_clock::to_time_t(timestampPoint);
        timestampStr = std::ctime(&timestampTime);
        timestampStr.pop_back(); // Remove newline character
        // Optionally truncate long messages for table formatting
        std::string truncatedTopic = (topicStr.size() > 20) ? topicStr.substr(0, 17) + "..." : topicStr;
        std::string truncatedMsg = (msgStr.size() > 20) ? msgStr.substr(0, 17) + "..." : msgStr;
        // Table formatting
        std::cout << "\033[2J\033[1;1H"; // Clear the screen
        std::cout << "\033[1;34m" << "ZMQ Subscriber " << socket_address << "  " << topic_string << "\033[0m" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << std::setw(20) << std::left << "Topic:" << std::setw(20) << std::left << truncatedTopic << std::endl;
        std::cout << std::setw(20) << std::left << "Message:" << std::setw(20) << std::left << truncatedMsg << std::endl;
        std::cout << std::setw(20) << std::left << "Timestamp:" << std::setw(20) << std::left << timestampStr << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << std::flush;
    }
    std::cout << "Subscriber shutdown completed." << std::endl;
    return 0;
}
