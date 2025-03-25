#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <stdlib.h>

#include <vector>
#include <string.h>
#include <stdexcept>
#include <syslog.h>

#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/ioctl.h>

#include <chrono>
#include <zmq.hpp>

#define DEBUG_MODE true

#define CMD_SRC_SOCKET "tcp://Rodrigos-MacBook-Air.local:5556"
#define MSG_PUB_ADDRESS "tcp://localhost:5555"

#define PC_VIDEO_ADDRESS "192.168.0.32"
#define SERCOMS_RX_BUFFER_SIZE 1024




float iir_filter(float input, float prev, float alpha) {
    return alpha * input + (1 - alpha) * prev;
}

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

std::string timestamp() {
    return std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
}


#endif // COMMON_H