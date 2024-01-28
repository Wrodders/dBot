#ifndef COMS_H
#define COMS_H

#include "../common/common.h"
#include "../drivers/serial.h"

/*
Communication Protocol:
Tranmission and reception are handled separately.
*************************************************
Tranmission: 
The Device; on which this firmware is executed, acts as a data publisher.
Unique Topics are used in a Pub Sub model where another program acts as a Subscriber.
It is the Subscribers responsibilty to correctly interpret the messages per topic.
The Device may only publish over a set list of topics, which can be activated/deactivated.
gstThe Device may list the format & description of each topic at start up and/or upon request.

Messages are framed as 
|-----+-----+-------+------+-------+- - - +-------+-------|
| SOF | LEN | DELIM |TOPIC | DELIM | DATA | DELIM | CKSUM |
| 1   | 2   | 1     | 2    | 1     | ...   | 1     | 2    |
|-----+-----+-------+------+-------+- - - +-------+-------|

SOF -- Start of Frame (0x3C / < )
LEN -- Size of Data (bytes)
TOPIC -- Hashed Topic String Identifier 
DATA -- LEN bytes of data
CKSUM -- 2 byte XOR 

Message Frames Have a Max Size

Topics must declare their message protocols in a format specifier string, e.g "%0.2f:%c:%d:%02d"
This will be used to encode a message into the frame, and allows the subscriber to correctly decode it. 

Subscribers will delimit messages based on receiving the start of frame byte, 
If a SOF is received before LEN bytes message is aborted and new message begins

*************************************************
Reception
Activating the Engineering Command program requires the reception of an Engineering Mode byte sequence. 
Exiting Engineering mode can only be done though reset. 

ID command runs upon entering engineering mode and reports all commands available on device and format as single source of truth. 
Commands consist of GET/SET/RUN modifiers which act on unique ids. 
Commands must register their input and response format.
All Commands have a timeout. Error messages are transmitted back over the Error topic. 
Parameters to be passed to commands may be specified as required or optional with default values

A successful command message will be parsed, executed, its responds over cmdResp topic and its acklogalged over cmdRet topic
Error in any part of this process will result in a Failed cmdRet and an accompanying error message 
over error topic identifiable by command id

*/


#define START_BYTE '<' // hex 0x3C 
#define STOP_BYTE  '>' // 
#define DELIM_BYTE ',' // hex 0x3A
#define TOPIC_SIZE 2
#define CKSUM_SIZE 2
#define MAX_MSG_DATA_SIZE 64
#define MSG_OVERHEAD_SIZE  sizeof(START_BYTE) + 2 + 1 + TOPIC_SIZE + 1 + CKSUM_SIZE
#define MAX_MSG_FRAME_SIZE MSG_OVERHEAD_SIZE + MAX_MSG_DATA_SIZE
 

#define QUEUE_SIZE 3

#define MAX_TOPICS 10
#define NAME_SIZE 10
#define MAX_DATA_ARGS 5
#define FORMAT_SIZE (MAX_DATA_ARGS * 2) + MAX_DATA_ARGS - 1


#define DEBUG "AA"



typedef struct MsgFrame{
    uint8_t size;
    char buf[MAX_MSG_FRAME_SIZE]; 
}MsgFrame;


typedef struct MsgTopic{
    char id[TOPIC_SIZE];
    char name[NAME_SIZE]; 
    char delim;
    char fmt[FORMAT_SIZE]; // argument format
    uint8_t numArgs; 
    bool active;
}MsgTopic;

typedef struct MsgHandler{
    uint8_t count;
    MsgTopic *topics[MAX_TOPICS];
}MsgHandler;

static MsgTopic initTopic(char *name, const char delim, char *format, uint8_t numArgs){
    MsgTopic tp;

    if(uCpy(tp.name, name, NAME_SIZE) == 0){tp.active = false;}

    if(numArgs > MAX_DATA_ARGS){tp.active = false;}
    tp.numArgs = numArgs;
    if(uCpy(tp.fmt, format, FORMAT_SIZE) == 0){tp.active = false;}
    tp.delim = delim;
    tp.active = true;
    return tp;
}


//****** Transmission Queue ***********************************************************//

QUEUE_DECLARATION(serTXQueue, MsgFrame, QUEUE_SIZE);
QUEUE_DEFINITION(serTXQueue, MsgFrame);

struct serTXQueue sertxQueue;

static void beginComs(void){
    serTXQueue_init(&sertxQueue);
    return;
}


// ****** Public API *****************************************************//

static void publish(const char *buf, uint8_t dataSize, MsgTopic *topic){
    //@Breif: Publishes Message over topic to Serial TX Queue
    #define START_IDX = 0,
    #define TOPIC_IDX = 1,
    #define SIZE_IDX = 3, // skip delim
    #define DATA_IDX = 5, //skip delim
    
    MsgFrame packet;
    uint8_t idx = 0;
    uint8_t payloadSize;
    if (dataSize > MAX_MSG_DATA_SIZE){
        payloadSize = MAX_MSG_DATA_SIZE;
        uCpy(topic->id, DEBUG, 2); // re route to debug topic on error
    }else{
        payloadSize = dataSize;
    }
    packet.buf[idx++] = START_BYTE;
    uCpy(&packet.buf[idx], topic->id, 2);

    packet.buf[idx++] = DELIM_BYTE;
    packet.buf[idx++] = payloadSize; 
    packet.buf[idx++] = DELIM_BYTE;
    for(int i = 0; i < payloadSize; i++){
        packet.buf[idx++] = buf[i];
    }
    packet.buf[idx++] = STOP_BYTE; 

    packet.size = idx;

    serTXQueue_enqueue(&sertxQueue,&packet);
    return;
}

static bool sendPckt(Serial *ser){
    //@Breif: Pulls message from queue and sends to UART buffer
    MsgFrame pckt;
    enum dequeue_result res = serTXQueue_dequeue(&sertxQueue,&pckt);
    if(res == DEQUEUE_RESULT_EMPTY){
        return false;
    }
    serialSend(ser, (uint8_t *)pckt.buf, pckt.size);
    return true;
}


#endif // COMMS_H