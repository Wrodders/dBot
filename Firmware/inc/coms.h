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
It is the Subscribers responsibilty to correctly interpret the messages per id.
The Device may only publish over a set list of topics, which can be activated/deactivated.
gstThe Device may list the format & description of each id at start up and/or upon request.

Messages are framed inside a packet:

|-------------Packet---------------|
|-----+-----+-------+- - - +-------+
| SOF | LEN | ID | DATA | EOF   |
| 1   | 1   | 1     | ...  | 1     |
|-----------+-------+- - - +-------+
      |--------Frame-------|


SOF -- Start of Frame 
LEN -- Size of Data (bytes)
ID -- Hashed Topic String Identifier 
DATA -- LEN bytes of data
EOF - End of Frame 

Message Frames Have a Max Size

Topics must declare their message protocols in a format specifier string, e.g "%0.2f:%c:%d:%02d"
This will be used to encode a message into the frame, and allows the subscriber to correctly decode it. 

Subscribers will delimit messages based on receiving the start of frame byte, 
If a SOF is received before LEN bytes message is aborted and new message begins

Data Received Byte By Byte in USART ISR
Put into Ringbuffer, serviced by Main through Non-Blocking GetMessage() returns
MsgFrame Struct used to Handle msgs by TopicIDs 


Data Passed to Queue of MessagesPackets
Queue serviced by Main every 100ms and written to TX ring buffer for transmission via ISR

*************************************************
Reception
Activating the Engineering Command program requires the reception of an Engineering Mode byte sequence. 
Exiting Engineering mode can only be done though reset. 

ID command runs upon entering engineering mode and reports all commands available on device and format as single source of truth. 
Commands consist of GET/SET/RUN modifiers which act on unique ids. 
Commands must register their input and response format.
All Commands have a timeout. Error messages are transmitted back over the Error id. 
Parameters to be passed to commands may be specified as required or optional with default values

A successful command message will be parsed, executed, its responds over cmdResp id and its acklogalged over cmdRet id
Error in any part of this process will result in a Failed cmdRet and an accompanying error message 
over error id identifiable by command id

*/


#define SOF_BYTE '<' // hex 0x3C 
#define EOF_BYTE  '\n' // 
#define DELIM_BYTE ':' // hex 0x3A
#define MAX_MSG_DATA_SIZE 64
#define MSG_OVERHEAD_SIZE  4 // SOF ID LEN EOF
#define MAX_MSG_FRAME_SIZE MSG_OVERHEAD_SIZE + MAX_MSG_DATA_SIZE
 
#define NUM_COMMANDS 10

#define ID_DEBUG 'A' // DEBUG Statements 
#define ID_CMD_RET 'B' // Return Value of Function
#define ID_IMU  'C' // PITCH ROLL YAW
#define ID_ODOM 'D' // Bot Velocity & Angle



typedef struct MsgFrame{
    uint8_t size; // current size of msg buffer
    uint8_t id; // id 
    uint8_t buf[MAX_MSG_FRAME_SIZE]; 
}MsgFrame;

typedef uint8_t (*Service)(MsgFrame *msg);

typedef struct Command{
    uint8_t id; 
    Service func;
}Command;

typedef struct CmdList{
    Command cmds[NUM_COMMANDS];
    uint8_t count;
    uint8_t retVal; // Store last return value of function executed
}CmdList;


typedef struct Coms{
    MsgFrame rxFrame;
    MsgFrame txFrame;
    CmdList server; // handles access to commands
}Coms;


// ******* Commander *************************************************************// 

static bool registerCmd(CmdList *cmdList, Service func){
    //Adds Cmd index by index in list
    uint8_t *cnt = &cmdList->count;
    if(cmdList->count == NUM_COMMANDS){return false;}
    Command *cmd = &cmdList->cmds[*cnt++];
    cmd->func = func;
    cmd->id = *cnt++;
    return true;
}

static bool comsGetMsg(Serial *ser, MsgFrame *msg){
    //@Grabs a message from the serial buffer is available
    //@Returns true Message Received
    static enum {IDLE, SIZE, ID, DATA, COMPLETE, ERROR} state = IDLE;
    while (rb_empty(&ser->rxRB) == 0){
        uint8_t dataIdx = 3; // start posiotn of data
        uint8_t byte;
        rb_get(&ser->rxRB, &byte); // pop byte
        switch (state){
            case IDLE:
                if(byte == SOF_BYTE){state=SIZE;}
                msg->size = 0; 
                break;
            case SIZE:
                msg->size = byte; 
                state = ID;
                break;
            case ID:
                msg->id = byte;
                state = DATA;
                break;
            case DATA:
                if(dataIdx > msg->size){state = ERROR; break;} // src overflow
                if(byte == SOF_BYTE){state = IDLE; break;} // multiple start bytes detected 
                if(byte == EOF_BYTE){state = COMPLETE; break;} // EOF
                msg->buf[dataIdx++] = byte; 
                break;
            case COMPLETE:
                USART_DR(USART1) = 'X' & USART_DR_MASK; // write byte
                state=IDLE; // reload 
                return true; // exit   
            case ERROR:
                msg->buf[0] = '\0';
                state=IDLE;
                return false; // exit 
        }
    }
    return false;
}

static bool comsCmdExec(CmdList *cmdList, MsgFrame *msg){
    //@Brief: LooksUp Cmd by ID, validates protocol
    //@Return: NULL if command not found

    if(msg->id > cmdList->count){return false;} //
    
    Command *cmd = &cmdList->cmds[msg->id];
    cmdList->retVal = cmd->func(msg);
    return true;
}


//****** Transmission Queue ***********************************************************//
static bool comsSendMsg(Serial *ser, char id, char *format, ...){
    //@Breif: Publishes Message over id to Serial TX Queue
    #define START_IDX  0
    #define LEN_IDX 1
    #define TOPIC_IDX  2
    #define DATA_IDX  3

    
    MsgFrame msg;
    va_list args;
    va_start(args, format);
    int len = mysprintf((char *)&msg.buf[DATA_IDX], 2, format, args); // ** NEED TO CHECK BUFFER SIZE
    va_end(args);
    if(len > MAX_MSG_DATA_SIZE - 1){ return false;} // msg data overflow
    uint8_t idx = 0;
    msg.buf[idx++] = SOF_BYTE;
    msg.buf[idx++] = len;
    msg.buf[idx++] = id;
    idx += len; // offset data
    msg.buf[idx++] = EOF_BYTE;
    serialSend(ser, msg.buf, idx);
    return true;
}


#endif // COMS_H