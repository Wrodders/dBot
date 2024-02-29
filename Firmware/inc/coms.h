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

|-------------Packet---------|
|-----+-----+----+- - - +----|
| SOF | LEN | ID | DATA | EOF|
| 1   | 1   | 1  | ...  | 1  |
|-----------+----+- - - +----|
      |-------Frame-----|


SOF -- Start of Frame 
LEN -- Size of Data (bytes)
ID -- Hashed Topic String Identifier 
DATA -- LEN bytes of data
EOF - End of Frame 


Message Frames Have a max size. 
Topics must declare their message protocols in a format specifier string, e.g "f:c:d:d" using the same format as printf and sprintf ect. 
This will be used to encode a message into the frame, and allows the subscriber to correctly decode it. 

Subscribers will delimit messages based on receiving the start of frame byte, 
If a SOF is received before LEN bytes message is aborted and new message begins. This acts as an escape/abort key. 

Data Received Byte By Byte in USART ISR
Put into Ringbuffer, serviced by Main through Non-Blocking GetMessage() returns a
MsgFrame Struct used to Handle msgs by TopicIDs 

Async messages passed to Queue of MessagesPackets
Queue serviced by Main every 100ms and written to TX ring buffer for transmission via ISR

*************************************************
Reception
In order ot solve the issue of dynamic discovery and possible miss match of commands and protocols between various systems
a variant of AT Commands logic is used. Devices have a default set of commands that can be used to identify the devices commands and protocols. 
ID command reports all commands available on device and format as single source of truth. 
Commands consist of GET/SET/RUN modifiers which act on unique ids mapped to pointers in an ordered array. 
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
 



typedef struct MsgFrame{
    uint8_t size; // current size of msg buffer
    uint8_t id; // id 
    uint8_t buf[MAX_MSG_FRAME_SIZE]; 
}MsgFrame;


typedef enum {
    PUB_CMD_RET = 0,
    PUB_ERROR,
    PUB_INFO, 
    PUB_DEBUG, 

    // ADD Application Specific Publishers
    PUB_IMU,
    PUB_TELEM,

    NUM_PUBS
}PUB_ID_t; // Publish Topic IDs

typedef enum {
    CMD_ID = 0,
    CMD_RESET,

    // ADD Application Specific Cmds 
    CMD_HELLO,

    NUM_CMDS
}CMD_ID_t; // Cmd Topic Ids


typedef struct {
    union {
        CMD_ID_t cmdId;
        PUB_ID_t pubId;
    }id;
    const char *name;
    const char *format;
}Topic;

const Topic pubMap[NUM_PUBS] = {
    {{.pubId = PUB_CMD_RET}, .name = "CMD_RET", .format ="%c:%s"}, // CMD_ID : RET_VAL
    {{.pubId = PUB_ERROR}, .name = "ERR", .format = "%s"}, // Error Message
    {{.pubId = PUB_INFO}, .name = "INFO", .format = "%s"}, // System INFO
    {{.pubId = PUB_DEBUG}, .name = "DBUG", .format = "%s"}, // Debug prints 

    // Application Publisher Topics 
    {{.pubId = PUB_IMU}, .name = "IMU", .format = "%f:%f:%f"}, // ROLL:PITCH:YAW
};

const Topic cmdMap[NUM_CMDS] = {
    {{.cmdId = CMD_ID }, .name = "IDENT", .format = "" },
    {{.cmdId = CMD_RESET}, .name = "RESET", .format = ""},

    // Application Cmd Topics
    {{.cmdId = CMD_HELLO}, .name = "HELLO", .format = ""}
};

typedef uint8_t (*Service)(MsgFrame *msg);

typedef struct Command{
    CMD_ID_t id; 
    Service func;
}Command;

typedef struct CmdList{
    Command cmds[NUM_CMDS];
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
    if(cmdList->count == NUM_CMDS){return false;}
    Command *cmd = &cmdList->cmds[*cnt++];
    cmd->func = func;
    cmd->id = *cnt++;
    return true;
}

static bool comsGetMsg(Serial *ser, MsgFrame *msg){
    //@Brief: Grabs a message from the serial buffer is available
    //@Note: Message Discarded if EOF received before declared msg len
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
    if(msg->id > cmdList->count){return false;} // no cmd implemented
    
    Command *cmd = &cmdList->cmds[msg->id];
    cmdList->retVal = cmd->func(msg);
    return true;
}



//****** Message Tranmission packetization  ***********************************************************//
/*
|-------------Packet---------|
|-----+-----+----+- - - +----|
| SOF | LEN | ID | DATA | EOF|
| 1   | 1   | 1  | ...  | 1  |
|-----------+----+- - - +----|
      |-------Frame-----|


SOF -- Start of Frame 
LEN -- Size of Data (bytes)
ID -- Hashed Topic String Identifier 
DATA -- LEN bytes of data
EOF - End of Frame 
*/

#define comsSendMsg(SER, ID, FORMAT, ... )                                   \
uint8_t DATA_IDX = 3;                                                        \
MsgFrame MSG;                                                                \
int SIZE = mysprintf((char *)&MSG.buf[DATA_IDX], 2, FORMAT, __VA_ARGS__ );   \
if(SIZE <= (MAX_MSG_DATA_SIZE - 1)){                                         \
uint8_t IDX = 0;                                                             \
MSG.buf[IDX++] = SOF_BYTE;                                                   \
MSG.buf[IDX++] = SIZE;                                                       \
MSG.buf[IDX++] = ID;                                                         \
IDX += SIZE;                                                                 \
MSG.buf[IDX++] = EOF_BYTE;                                                   \
serialSend(SER, MSG.buf, IDX);                                               \
}                                                                           


#endif // COMS_H