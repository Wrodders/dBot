#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <arpa/inet.h>
#include <poll.h>

void printUsage() {
    printf("Usage: serial_udp_bridge <serial_port> <baud_rate>\n");
}

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

int main(int argc, char *argv[]) {
    // Check if the correct number of arguments are provided
    if (argc != 3) {
        printUsage();
        return 1;
    }

    const char *serialPortName = argv[1];
    int baudRate = atoi(argv[2]);

    // Open the serial port
    int serialPort = open(serialPortName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        perror("Error opening serial port");
        return 1;
    }

    // Configure the serial port
    if (configureSerialPort(serialPort, baudRate) == -1) {
        close(serialPort);
        return 1;
    }
    // Set up poll structure
    struct pollfd pollFds[2];
    pollFds[0].fd = serialPort;
    pollFds[0].events = POLLIN;

    // Read from the serial port and forward to UDP
    char buffer[256];
    while (1) {
        if (poll(pollFds, 2, -1) > 0) {
            if (pollFds[0].revents & POLLIN) {
                ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer) - 1);
                if (bytesRead > 0) {
                    buffer[bytesRead] = '\0'; // Null-terminate the string
                    printf("%s", buffer);
                } else if (bytesRead < 0) {
                    perror("Error reading from serial port");
                    break;
                }
            }
        } else {
            perror("Error in poll");
            break;
        }
    }

    // Close sockets and serial port
    close(serialPort);

    return 0;
}
