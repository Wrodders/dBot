#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

// Here we could simulate nodes publishing telemetry data and responding to commands
std::thread node_simulation([&]() {
    // Simulate a node (e.g., IMU node)
    std::string bot_cmd_router_address = "ipc:///tmp//bot_cmds";
    std::string bot_telem_pub_address = "ipc:///tmp//bot_telem";

    zmq::context_t context(1);
    zmq::socket_t nodeCmdDealer(context, ZMQ_DEALER);  // Node dealer socket
    nodeCmdDealer.connect(bot_cmd_router_address);

    zmq::socket_t nodeTelemPub(context, ZMQ_PUB);  // Node telemetry publisher
    nodeTelemPub.connect(bot_telem_pub_address);

    while (true) {
        // Simulate receiving commands from external source
        zmq::message_t cmd_msg;
        nodeCmdDealer.recv(&cmd_msg);
        std::string command = cmd_msg.to_string();
        std::cout << "Received command: " << command << std::endl;

        // Simulate publishing telemetry data
        zmq::message_t telem_msg("IMU Data: [x:1.0, y:2.0, z:3.0]");
        nodeTelemPub.send(telem_msg, zmq::send_flags::none);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
});


int main(int argc, char** argv) {
    std::string bot_cmd_router_address = "ipc:///tmp//bot_cmds";
    std::string bot_telem_sub_address = "ipc:///tmp//bot_telem";
    std::string ext_cmd_router_address = "tcp://*:5556";
    std::string ext_telem_pub_address = "tcp://*:5555";

    if (argc > 1) bot_cmd_router_address = argv[1];
    if (argc > 2) bot_telem_sub_address = argv[2];
    if(argc > 3) ext_cmd_router_address = argv[3];
    if(argc > 4) ext_telem_pub_address = argv[4];

    std::cout << "Command Router Address: " << bot_cmd_router_address << std::endl;
    std::cout << "Telemetry Subscriber Address: " << bot_telem_sub_address << std::endl;

    zmq::context_t context(1);
    // Command Router
    zmq::socket_t frontend(context, ZMQ_ROUTER);  // Command Router for IPC
    frontend.bind(bot_cmd_router_address);
    // Telemetry 
    zmq::socket_t telemetrySub(context, ZMQ_SUB);  // Telemetry Publisher
    telemetrySub.bind(bot_telem_sub_address);

    // External Command Router
    zmq::socket_t externalCmdRouter(context, ZMQ_ROUTER);  // External Command Router
    externalCmdRouter.bind(ext_cmd_router_address);
    // External Telemetry Publisher
    zmq::socket_t externalTelemPub(context, ZMQ_PUB);  // External Telemetry Publisher
    externalTelemPub.bind(ext_telem_pub_address);
    // handle commands between intaentla ned external routers, proxy the ipc ad tcp telemtry data
    zmq::proxy(frontend, externalCmdRouter, nullptr); // Proxy commands between internal and external routers
    zmq::proxy(telemetrySub, externalTelemPub, nullptr); // Proxy telemetry data between internal and external publishers

    return 0;
}
