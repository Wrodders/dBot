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

A successful command message will be parsed, executed, its responds over cmdResp id and its acknowledge over cmdRet id
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
    bool valid;
    uint8_t size; // current size of msg buffer
    uint8_t id; // id 
    uint8_t buf[MAX_MSG_DATA_SIZE]; 
}MsgFrame;


typedef enum {
    PUB_CMD_RET = 0,
    PUB_ERROR,
    PUB_INFO, 
    PUB_DEBUG, 

    // ADD Application Specific Publishers
    PUB_IMU,
    PUB_ODOM,

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
    const char* name;
    const char* format;
}Topic;


typedef struct Coms{

    MsgFrame rxFrame;
    MsgFrame txFrame;

    const Topic* pubMap;
    const Topic* cmdMap;
}Coms;

static Coms comsInit(const Topic* pubMap, const Topic* cmdMap){
    Coms c = { .pubMap = pubMap, .cmdMap = cmdMap};
    return c;
}


static inline const char* comsGetPubFmt(Coms* coms, PUB_ID_t ID){return coms->pubMap[ID].format;}
static inline const char* comsGetPubName(Coms* coms, PUB_ID_t ID){return coms->pubMap[ID].name;}
static inline const char* comsGetCmdFmt(Coms* coms, CMD_ID_t ID){return coms->cmdMap[ID].format;}
static inline const char* comsGetCmdName(Coms* coms, CMD_ID_t ID){return coms->cmdMap[ID].name;}

//****** Message Tranmission packetization  ***************//
/*
|-------Packet---------|
|-----+----+- - - +----|
| SOF | ID | DATA | EOF|
| 1   | 1  | .... | 1  |
|-----+----+- - - +----|
      |------Frame-----|
SOF -- Start of Frame 
ID --  Topic Identifier 
DATA -- LEN bytes of data
EOF - End of Frame 
*/
static bool comsSendMsg(Coms* coms, Serial* ser, PUB_ID_t ID, ...){
    //@Brief: Formats and Packets a Message, sends over serial. 
    const uint8_t DATA_IDX = 2;
    uint8_t* msgBuf = coms->txFrame.buf; // readability
    const char* pubFmt = comsGetPubFmt(coms, ID);

    va_list args;
    va_start(args, ID);
    const size_t dataSize = vsnprintf((char*)&msgBuf[DATA_IDX],MAX_MSG_DATA_SIZE, pubFmt, args); 
    va_end(args);
    if(dataSize > MAX_MSG_DATA_SIZE - 1 ){return false;}
    size_t idx = 0;
    msgBuf[idx++] = SOF_BYTE;
    msgBuf[idx++] = ID + 'a';
    idx += dataSize;
    msgBuf[idx++] = EOF_BYTE;
    serialSend(ser, msgBuf, idx);


    return true;
}




static void comsDeclarePubs(Coms* coms, Serial* ser){

    const uint8_t DATA_IDX = 2;
    uint8_t* msgBuf = coms->txFrame.buf; // readability
    const Topic* pubMap = coms->pubMap;
    const Topic* cmdMap = coms->cmdMap;
    size_t idx;
    // Pack Pub Topic Info: Name:Format
    for(int i = 0; i < NUM_PUBS; i++){
        idx = 0;
        msgBuf[idx++] = SOF_BYTE;
        msgBuf[idx++] = PUB_CMD_RET + 'a';
        msgBuf[idx++] = CMD_ID; // Calling Cmd IDmake
        msgBuf[idx++] = ':'; // delim

        const char* pubName = comsGetPubName(coms, i);
        idx += uCpy((char*)&msgBuf[idx],pubName, 0); 
        msgBuf[idx++] = ':';
        const char* pubFmt = comsGetPubFmt(coms, i);
        idx += uCpy((char*)&msgBuf[idx], pubFmt, 0);
        msgBuf[idx++] = EOF_BYTE;
        serialSend(ser, msgBuf, idx);
    }
}


#endif // COMS_H