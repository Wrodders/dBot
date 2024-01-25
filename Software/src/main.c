#include "../common/common.h"
#include "../inc/serialComs.h"



void printUsage() {
    printf("Usage: serial_udp_bridge <serial_port> <baud_rate>\n");
}

int main(int argc, char **argv){
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

    // Read from the serial port output to stdout 
    char buffer[256];
    fflush(stdout);
    while (1) {
        if (poll(pollFds, 2, -1) > 0) {
            if (pollFds[0].revents & POLLIN) {
                ssize_t bytesRead = read(serialPort, buffer, sizeof(buffer) - 1);
                if (bytesRead > 0) {
                    buffer[bytesRead] = '\0'; // Null-terminate the string
                    fputs(buffer, stdout);
                    fflush(stdout);
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