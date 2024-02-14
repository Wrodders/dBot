#include <zmq.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

int main(void) {
    // Initialize context and socket
    void *context = zmq_ctx_new();
    void *subscriber = zmq_socket(context, ZMQ_SUB);

    // Connect to the Pi Zero's TCP port for telemetry
    zmq_connect(subscriber, "tcp://192.168.1.253:5555"); // IP address and port of the Pi Zero

    // Subscribe to all messages
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    // Main loop to receive telemetry and send commands
    while (1) {
        // Receive telemetry
        char msgFrame[256];
        int recv_size = zmq_recv(subscriber, msgFrame, 255, 0);
        if (recv_size != -1) {
            // Null-terminate the received telemetry
            msgFrame[recv_size] = '\0';
            fprintf(stdout,"Received msgFrame: %s\n", msgFrame);
        }
        else {
            fprintf(stderr, "Error receiving msgFrame\n");
            break;
        }

        // Example command (replace with actual command)
        char command[] = "Example command from PC";

        // Send command
        void *requester = zmq_socket(context, ZMQ_REQ);
        zmq_connect(requester, "tcp://192.168.1.253:5556"); // Port number 5556 for command transmission

        zmq_send(requester, command, strlen(command), 0);

        char reply[256];
        zmq_recv(requester, reply, 255, 0);
        printf("Received reply: %s\n", reply);

        zmq_close(requester);
    }

    // Close socket and destroy context
    zmq_close(subscriber);
    zmq_ctx_destroy(context);

    return 0;
}
