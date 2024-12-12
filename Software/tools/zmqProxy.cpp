#include <zmq.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <unistd.h>


int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_socket1> <input_socket2> <output_socket> \n";
        return 1;
    }
    std::string input_socket1 = argv[1];
    std::string input_socket2 = argv[2];
    std::string output_socket = argv[3];

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    subscriber.connect(input_socket1);
    subscriber.connect(input_socket2);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(output_socket);
    std::cout << "Proxy started.\n";
    std::cout << "+-------------------+-------------------+\n";
    std::cout << "|     Socket Type   |      Address      |\n";
    std::cout << "+-------------------+-------------------+\n";
    std::cout << "| Input Socket 1    | " << input_socket1 << " |\n";
    std::cout << "| Input Socket 2    | " << input_socket2 << " |\n";
    std::cout << "| Output Socket     | " << output_socket << " |\n";
    std::cout << "+-------------------+-------------------+\n";

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
