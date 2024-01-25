#ifndef ZMQ_BRIDGE_H
#define ZMQ_BRIDGE_H

#include "../common/common.h"

#include <czmq.h>


int main(int argc, char **argv){

    zsock_t *responder = zsock_new(ZMQ_REP);
    int r = zsock_bind(responder, "tcp://localhost:5555");
    if(r != 5555){
        printf("Failed ot bind to port\n");
    }
    
    while(true){
        char *msg = zstr_recv(responder);
        if(!strcmp(msg, "Hello")){
            zstr_send(responder, "World\n");
        }
        free(msg);
    }
    

}

#endif // ZMQ_BRIDGE