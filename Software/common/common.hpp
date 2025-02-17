#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <ctime>

#include <string.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>

#include <zmq.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


#define CMD_SOCKET_ADDRESS "ipc:///tmp/botcmds"
#define MSG_PUB_ADDRESS "ipc:///tmp/botmsgs"

#define PC_VIDEO_ADDRESS "192.168.0.32"
#define SERCOMS_RX_BUFFER_SIZE 1024

//@Brief: Format timestamps in human-readable format
std::string formatTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp) {
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    std::tm tm = *std::localtime(&time_t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count() % 1000000000;
    oss << "." << std::setw(9) << std::setfill('0') << ns;
    return oss.str();
}


#endif // COMMON_H