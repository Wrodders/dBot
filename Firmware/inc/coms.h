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
The Device may list the format & description of each topic at start up and/or upon request.

Messages are framed as 
|-----+-----+-------+- - - +-------|
| SOF | LEN | TOPIC | DATA | CKSUM |
| 1   | 2   | 2     | ...  | 2     | 
|-----+-----+-------+- - - +-------|

SOF -- Start of Frame (0x3C / < )
LEN -- Size of Data (bytes)
TOPIC -- Hashed Topic String Identifier 
DATA -- LEN bytes of data
CKSUM -- 2 byte XOR 

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

A succesful command message will be parsed, executed, its responds over cmdResp topic and its acklogalged over cmdRet topic
Error in any part of this process will result in a Failed cmdRet and an accompanying error message 
over error topic identifiable by command id


*/



#define QUEUE_SIZE 3
#define START_BYTE '<' // hex 0x3C 
#define STOP_BYTE  '>' // 
#define DELIM_BYTE ',' // hex 0x3A
#define MSG_TOPIC_SIZE 2
#define MSG_HEADER_SISE  MSG_TOPIC_SIZE + 1 + 1  //TOPIC Delim SIZE
#define MAX_MSG_DATA_SIZE 54 
#define MAX_MSG_PACKET_SIZE (sizeof(START_BYTE) + MSG_HEADER_SISE + sizeof(DELIM_BYTE) + MAX_MSG_DATA_SIZE + sizeof(DELIM_BYTE) + sizeof(STOP_BYTE))

typedef enum MsgState{
    MSG_WAIT = 0,
    MSG_START,
    MSG_ID,
    MSG_SIZE,
    MSG_DATA,
    MSG_COMPLETE
}MsgState;

typedef struct Message{
    MsgState state;
    char topic[2]; 
    uint8_t size;
    char data[MAX_MSG_DATA_SIZE];
}Message;

typedef struct MsgPacket{
    uint8_t size;
    char buf[MAX_MSG_PACKET_SIZE]; 
}MsgPacket;

typedef enum MsgTopic{
    DEBUG,
    IMU,
    SPEED,
    VBAT
}MsgTopic;


// ********* Command Interface *******************************************//




//****** Queue ***********************************************************//

QUEUE_DECLARATION(serTXQueue, MsgPacket, QUEUE_SIZE);
QUEUE_DEFINITION(serTXQueue, MsgPacket);

struct serTXQueue sertxQueue;

static void beginComs(void){
    serTXQueue_init(&sertxQueue);
    return;
}


// ****** Public API *****************************************************//

static void publish(const char *buf, uint8_t dataSize, MsgTopic topic){
    //@Breif: Publishes Message over topic to Serial TX Queue
    #define START_IDX = 0,
    #define TOPIC_IDX = 1,
    #define SIZE_IDX = 3, // skip delim
    #define DATA_IDX = 5, //skip delim
    
    MsgPacket packet;
    uint8_t idx = 0;
    uint8_t payloadSize;
    if (dataSize > MAX_MSG_DATA_SIZE){
        payloadSize = MAX_MSG_DATA_SIZE;
        topic = DEBUG; // redirect to error handler
    }else{
        payloadSize = dataSize;
    }
    packet.buf[idx++] = START_BYTE;
    packet.buf[idx++] = topic;
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
    MsgPacket pckt;
    enum dequeue_result res = serTXQueue_dequeue(&sertxQueue,&pckt);
    if(res == DEQUEUE_RESULT_EMPTY){
        return false;
    }
    serialSend(ser, (uint8_t *)pckt.buf, pckt.size);
    return true;
}


#endif // COMMS_H