#ifndef SERIAL_COMS_H
#define SERIAL_COMS_H

#include "../common/common.h"
#include <termios.h>

int configureSerialPort(int serialPort, int baudRate) {
    // Configure the serial port settings
    struct termios serialConfig;
    if (tcgetattr(serialPort, &serialConfig) == -1) {
        perror("Error getting serial port attributes");
        return -1;
    }

    cfsetispeed(&serialConfig, baudRate);
    cfsetospeed(&serialConfig, baudRate);
    serialConfig.c_cflag |= (CLOCAL | CREAD);
    serialConfig.c_cflag &= ~PARENB;
    serialConfig.c_cflag &= ~CSTOPB;
    serialConfig.c_cflag &= ~CSIZE;
    serialConfig.c_cflag |= CS8;

    if (tcsetattr(serialPort, TCSANOW, &serialConfig) == -1) {
        perror("Error setting serial port attributes");
        return -1;
    }

    return 0;
}


#endif // SERIAL_COMS_H