#include <zmq.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "../../common/serialComs.h"



void printUsage() {
    printf("Usage: serial_udp_bridge <serial_port> <baud_rate>\n");
}

int main(int argc, char **argv) {

    if (argc != 3) {
        printUsage();
        return -1;
    }
    // get cli parameters
    const char *serialPortName = argv[1];
    int baudRate = atoi(argv[2]);
    // *** SETUP **** // 
    // Initialize zmq context and socket
    const char *address = "tcp://*:5555";
    void *context = zmq_ctx_new();
    void *publisher = zmq_socket(context, ZMQ_PUB);
    if (zmq_bind(publisher, address) == 0){
        fprintf(stderr, "Error in Binding PUB to: %s", address);
    }

    
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
            zmq_send(publisher, buffer, strlen(buffer), 0);
            fflush(stdout);
        }
    }

    sp_close(port);
    if(port != NULL){sp_free_port(port);}

    // Close socket and destroy context
    zmq_close(publisher);
    zmq_ctx_destroy(context);

    return 0;
}
