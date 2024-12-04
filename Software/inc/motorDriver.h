#ifndef MOTOR_CNTRL_H
#define MOTOR_CNTRL_H

// Motor Controller Driver Functionality
#include "../common/common.h"

enum PUBS{
    PUB_CMD_RET = 0,
    PUB_INFO,
    PUB_DEBUG,
    PUB_ERROR,
    PUB_STATE,
    PUB_NUMS
};

struct MotorState{
    struct {
        float pitch, roll;
    } imu;
    struct {
        float lw, rw;
        float linVel, angVel;
        float xPos, yPos, theta;
    } odom;
    struct {
        float tl, tr, ul,ur,ub,ua,uv;
    } cntrl;

    time_t timestamp;
    float cmdRet;
    char info[256], debug[256], error[256];
};


int configure_serial_port(const char *port) {
    struct termios options;
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    // Get current options
    if (tcgetattr(fd, &options) < 0) {
        perror("Error getting serial port attributes");
        close(fd);
        return -1;
    }

    // Set baud rate (example: 9600)
    cfsetispeed(&options, B115200);    // Input baud rate
    cfsetospeed(&options, B115200);    // Output baud rate

    // Set 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB;      // No parity
    options.c_cflag &= ~CSTOPB;      // 1 stop bit
    options.c_cflag &= ~CSIZE;       // Mask data bits
    options.c_cflag |= CS8;          // 8 data bits

    // Set canonical mode (line buffering)
    options.c_lflag |= ICANON;       // Enable canonical mode
    options.c_lflag |= ECHO;         // Enable echo
    options.c_lflag |= ECHOE;        // Enable erasure
    options.c_lflag |= ISIG;         // Enable signal processing (Ctrl+C, Ctrl+Z, etc.)

    // Disable non-canonical modes like raw input or extended input processing
    options.c_lflag &= ~(ICRNL | INLCR); // No newline translation

    // Set timeout (for example, 5 seconds)
    options.c_cc[VMIN] = 1;          // Minimum number of characters for non-blocking read
    options.c_cc[VTIME] = 50;        // Timeout in deciseconds (500ms)

    // Apply the settings to the serial port
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("Error setting serial port attributes");
        close(fd);
        return -1;
    }

    return fd;
}


#endif  // ßMOTOR_CNTRL_Hß