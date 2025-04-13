#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <stdlib.h>

#include <vector>
#include <array>
#include <string.h>
#include <stdexcept>
#include <syslog.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <chrono>


#define DEBUG_MODE true

#define CMD_SRC_SOCKET "tcp://Rodrigos-MacBook-Air.local:5556"
#define MSG_PUB_ADDRESS "tcp://localhost:5555"

#define PC_VIDEO_ADDRESS "192.168.0.32"

float iir_lpf(float input, float prev, float alpha) {
    return alpha * input + (1 - alpha) * prev;
}

#endif // COMMON_H