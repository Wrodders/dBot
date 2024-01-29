#include "../common/common.h"
#include "../inc/serialComs.h"


void printUsage() {
    printf("Usage: serial_udp_bridge <serial_port> <baud_rate>\n");
}

int main(int argc, char **argv){

    if (argc != 3) {
        printUsage();
        return 1;
    }
    // get cli parameters
    const char *serialPortName = argv[1];
    int baudRate = atoi(argv[2]);
    
    struct sp_port *port;
    if(scanPorts(serialPortName, &port) < 0){return -1;}

    // open & config port
    if (initSerialPort(port,baudRate) != 0){
        if(DEBUG){printf("error in config\n");}
        return -1;
    }
    

    sp_flush(port, SP_BUF_INPUT);
    while (1) {
        char buffer[256];
        int len = sp_nonblocking_read(port, buffer, sizeof(buffer)); // Read data from the serial port

        if (len > 0) {
            buffer[len] = '\0'; // Null-terminate the received data
            printf("%d:%s", len, buffer);
            fflush(stdout);
        }
    }

    sp_close(port);
    if(port != NULL){sp_free_port(port);}

    return 0;
}