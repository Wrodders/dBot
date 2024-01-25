#ifndef SERIAL_COMS_H
#define SERIAL_COMS_H

#include "../common/common.h"
#include <libserialport.h>


static const int DEBUG = 1;

int scanPorts(const char *portName, struct sp_port **port) {
    struct sp_port **ports;
    int retval = sp_list_ports(&ports);

    if (retval == SP_OK) {
        for (int i = 0; ports[i]; i++) {
            const char *currentPortName = sp_get_port_name(ports[i]);
            if(DEBUG){printf("Found port: '%s'\n", currentPortName);}
            if (strcmp(currentPortName, portName) == 0) {
                sp_copy_port(ports[i], port); // match
                sp_free_port_list(ports);
                return 0;
            }
        }
        // Port name not found in the list
        if(DEBUG){printf("No matching port found for %s\n", portName);}
        *port = NULL;
        sp_free_port_list(ports);
    } else {
        if(DEBUG){printf("No serial devices detected\n");}
    }

    return -1;
}



int initSerialPort(struct sp_port *port, int baudrate){
    //@Brief: Configures serial port parameters
    //@Returns: -1 on failure;
    int retval = 0; 
    const int bits = 8;
    const int stopBits = 1;
    const int parity = SP_PARITY_NONE;
    
    if(sp_open(port, SP_MODE_READ) != SP_OK){retval  = -1;}
    else if(sp_set_baudrate(port, baudrate) != SP_OK){retval = -1;}
    else if(sp_set_bits(port, bits) != SP_OK){retval = -1;}
    else if(sp_set_stopbits(port, stopBits) != SP_OK){retval = -1;}
    else if(sp_set_parity(port, parity) != SP_OK){retval = -1;}
    
    return retval;
}



#endif // SERIAL_COMS_H