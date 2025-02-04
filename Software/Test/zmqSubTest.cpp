
#include <zmq.hpp>
#include <iostream>
#include <getopt.h>
#include <csignal>
#include <iomanip>
#include <atomic>

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

    std::cout << "Subscribed to " << socket_address << " on topic \"" << topic_string << "\"" << std::endl;

    while (true) {
        zmq::message_t topic, msg;
        
        zmq::recv_result_t result_topic = subSocket.recv(topic, zmq::recv_flags::none);
        if (!result_topic) {
            std::cerr << "Failed to receive topic" << std::endl;
            break;
        }

        zmq::recv_result_t result_msg = subSocket.recv(msg, zmq::recv_flags::none);
        if (!result_msg) {
            std::cerr << "Failed to receive message" << std::endl;
            break;
        }

        std::string topicStr(static_cast<char *>(topic.data()), topic.size());
        std::string msgStr(static_cast<char *>(msg.data()), msg.size());

        // Optionally truncate long messages for table formatting
        std::string truncatedTopic = (topicStr.size() > 20) ? topicStr.substr(0, 17) + "..." : topicStr;
        std::string truncatedMsg = (msgStr.size() > 20) ? msgStr.substr(0, 17) + "..." : msgStr;

       

        std::cout << "\033[1;34m" << "ZMQ Subscriber " << socket_address << "  " << topic_string << "\033[0m" << std::endl;
        std::cout << "\033[1;36m" << "+----------------------+----------------------+" << "\033[0m" << std::endl;
        std::cout << "\033[1;36m" << "| Topic                | Message              |" << "\033[0m" << std::endl;
        std::cout << "\033[1;36m" << "+----------------------+----------------------+" << "\033[0m" << std::endl;
        std::cout << "| " << std::left << std::setw(20) << truncatedTopic << " | " << std::setw(20) << truncatedMsg << " |" << std::endl;
        std::cout << "\033[1;36m" << "+----------------------+----------------------+" << "\033[0m" << std::endl;
    }

    std::cout << "Subscriber shutdown completed." << std::endl;
    return 0;
}
