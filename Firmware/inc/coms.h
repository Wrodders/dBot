#ifndef COMS_H
#define COMS_H

#include "../common/common.h"
#include "../drivers/serial.h"

/*
|-----------Packet---------|
|-----+---+----+- - - +----|
| SOF |CMD| ID | DATA | EOF|
| 1   | 1 | 1  | .... | 1  |
|-----+---+----+- - - +----|
      |--------Frame-------|
SOF -- Start of Frame
CMD -- CMD Type Get Set Run
ID --  Topic Identifier 
DATA -- LEN bytes of data
EOF - End of Frame 
*/

#define SOF_BYTE '<' // hex 0x3C 
#define EOF_BYTE  '\n' // 
#define DELIM_BYTE ':' // hex 0x3A
#define MAX_MSG_DATA_SIZE 64
#define MSG_OVERHEAD_SIZE  4 // SOF ID LEN EOF
#define MAX_MSG_FRAME_SIZE MSG_OVERHEAD_SIZE + MAX_MSG_DATA_SIZE
#define PROT_CMD_IDX 0 
#define PROT_ID_IDX 1

typedef struct MsgFrame{
    uint8_t size; // current size of msg buffer
    uint8_t cmdType;
    uint8_t id;  // id for map  
    uint8_t buf[MAX_MSG_DATA_SIZE]; 
}MsgFrame;

typedef enum {
    CMD_GET = 0, // Get Parameter Value
    CMD_SET,     // Set Parameter Value
    CMD_RUN,      // Execute Function
    NUM_CMDS
}CMD_t; // Command Type

typedef enum {
    PUB_CMD_RET = 0,
    PUB_ERROR,
    PUB_INFO, 
    PUB_DEBUG, 
    // ADD Application Specific Publishers
    PUB_IMU,
    PUB_ODOM,
    PUB_CTRL,

    NUM_PUBS
}PUB_ID_t; // Publish Topic IDs

typedef enum {
    PRM_ID = 0,
    // Motor Speed PID 
    PRM_LT, PRM_LP, PRM_LI,
    PRM_RT, PRM_RP, PRM_RI,
    // Balance PID
    PRM_BT,
    PRM_BP,
    PRM_BI,
    PRM_BD,
    // AngVel Offset
    PRM_VT,
    PRM_VP,
    PRM_VI,
    PRM_VD,
    PRM_VA,
    PRM_AT,
    PRM_AP,
    PRM_AI,
    PRM_AD,
    PRM_AA,
    NUM_PARAMS
}PARAM_ID_t; // Parameter Ids

typedef enum {
    RUN_RESET = 0,
    RUN_UPGRADE,

    NUM_RUN
}RUN_ID_t; // Run Command Id

typedef struct {
    PUB_ID_t id;
    const char* name;
    const char* format; 
    const uint8_t nArgs;
}Topic;

typedef struct {
    PARAM_ID_t id;
    const char* name;
    const char* format;
    float *const param;
}Param;

typedef struct {
    RUN_ID_t id;
    const char* name;
    const char* format;
    const uint8_t nArgs;
}RunFunc;

typedef enum{
    IDLE = 0, 
    CMDTYPE,
    ID, 
    DATA, 
    ERROR
}COMS_STATE_t;

typedef struct Coms{
    COMS_STATE_t state;
    MsgFrame rxFrame;
    MsgFrame txFrame;

    const Topic* pubMap;
    const Param* paramMap;
}Coms;

static Coms comsInit(const Topic* pubMap, const Param* paramMap){
    Coms c = { .pubMap = pubMap, .paramMap = paramMap};
    return c;
}

static inline const char* comsGetPubFmt(Coms* coms, PUB_ID_t pubID){return coms->pubMap[pubID].format;}
static inline const char* comsGetPubName(Coms* coms, PUB_ID_t pubID){return coms->pubMap[pubID].name;}
static inline const char* comsGetCmdFmt(Coms* coms, PARAM_ID_t pubID){return coms->paramMap[pubID].format;}
static inline const char* comsGetCmdName(Coms* coms, PARAM_ID_t pubID){return coms->paramMap[pubID].name;}

//****** Message Tranmission packetization  ***************//

static bool comsSendMsg(Coms* coms, Serial* ser, PUB_ID_t pubID, ...){
    //@Brief: Formats and Packets a Message, sends over serial. 
    const uint8_t DATA_IDX = 2;
    uint8_t* msgBuf = coms->txFrame.buf; // readability
    const char* pubFmt = comsGetPubFmt(coms, pubID);

    va_list args;
    va_start(args, pubID);
    const size_t dataSize = vsnprintf((char*)&msgBuf[DATA_IDX],MAX_MSG_DATA_SIZE, pubFmt, args); 
    va_end(args);
    if(dataSize > MAX_MSG_DATA_SIZE - 1 ){return false;}
    size_t idx = 0;
    msgBuf[idx++] = SOF_BYTE;
    msgBuf[idx++] = pubID + 'a';
    idx += dataSize;
    msgBuf[idx++] = EOF_BYTE;
    serialSend(ser, msgBuf, idx);
    return true;
}

//@Brief: Processes Received Data Into Message Frame
static bool comsGrabMsg(Coms* coms, Serial* ser){
    uint8_t byte;
    while(rbGet(&ser->rxRB,  &byte) == true){
        switch(coms->state){
            case IDLE:
                if(byte == SOF_BYTE){
                    coms->state=CMDTYPE;
                    coms->rxFrame.size = 0;
                    memset(coms->rxFrame.buf, 0, MAX_MSG_DATA_SIZE);
                }
                break;
            case CMDTYPE:
                if(byte - 'a' >= NUM_CMDS){coms->state=ERROR; break;} // Failed to match Command
                coms->rxFrame.cmdType = byte - 'a';
                coms->state=ID;
                break;
            case ID:
                coms->rxFrame.id = byte - 'a';
                coms->state = DATA;
                break;
            case DATA:
                if(byte == SOF_BYTE){coms->state=ERROR;break;} // Early Escape
                if(coms->rxFrame.size == MAX_MSG_DATA_SIZE){coms->state=ERROR;break;} 
                if(byte == EOF_BYTE){
                    coms->rxFrame.buf[coms->rxFrame.size++]='\0'; // null term char array!
                    coms->state=IDLE;
                    return true;
                }
                coms->rxFrame.buf[coms->rxFrame.size++]=byte;
                break;
            case ERROR:
                coms->state=IDLE;
                break;
            default:
                coms->state=IDLE;
                break;
        }
    }
    return false;
}

static void comsProcessMsg(Coms * coms,Serial *ser){
    uint8_t paramIdx = coms->rxFrame.id;
    if(paramIdx >= NUM_PARAMS){
        comsSendMsg(coms, ser,PUB_ERROR, "INVALID PARAM");
        return;
    }
    if(coms->paramMap[paramIdx].param == NULL){
        comsSendMsg(coms, ser, PUB_ERROR, "INVALID ACCESS");
        return;
    }
    switch(coms->rxFrame.cmdType){
        case CMD_GET:
            char valBuf[7]; // working buffer
            snprintf(valBuf, sizeof(valBuf), "%0.3f", *coms->paramMap[paramIdx].param);
            comsSendMsg(coms, ser, PUB_CMD_RET,(paramIdx + 'a'),valBuf);
            break;
        case CMD_SET:
            float val; // working variable
            val = atof((char*)coms->rxFrame.buf); // read in paramater value from start of data
            *coms->paramMap[paramIdx].param = val; // lookup into param Map 
            break;
        case CMD_RUN:
            comsSendMsg(coms, ser, PUB_DEBUG, "NotImp");
            break;
        default:
            comsSendMsg(coms, ser, PUB_ERROR, "INVALID CMDTYPE");
            break;
    }
}

#endif // COMS_H