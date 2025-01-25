#ifndef COMS_H
#define COMS_H

#include "../common/common.h"
#include "../drivers/serial.h"

#include "mcuComs.h"
#include "../common/errorCodes.h"


struct MsgFrame{
    size_t bufSize; // current size of msg buffer
    uint8_t cmdID; // Command Type identifier
    uint8_t id;  // Register id for map  
    uint8_t buf[MAX_MSG_FRAME_SIZE]; 
}MsgFrame;

struct Topic {
    enum Publishers id;       // Topic Register ID
    const char* name;       // Topic Register Name
    const char* format;     // Topic Register Format
    const uint8_t nArgs;    // Number of Arguments
}Topic;

struct Param {
    enum ParamRegisters id;    // Param Register ID
    const char* name;    // Param Register Name
    const char* format;  // Param Register Format
    float *const param;  // Pointer to Param Variable
}Param;

struct RunFunc{
    enum RemoteProcedures id; // Remote Procedure ID
    const char* name;         // Remote Procedure Name
    const char* format;       // Argument Format
    const uint8_t nArgs;      // Number of Arguments
    const void* func;         // Function Pointer
}RunFunc;

enum COMS_STATE_t{
    IDLE = 0, 
    CMDTYPE,
    ID, 
    DATA, 
    ERROR
}COMS_STATE_t;



struct Coms{
    enum COMS_STATE_t state;
    struct MsgFrame rxFrame; // Reception Protocol Frame
    struct MsgFrame txFrame; // Transmission Protocol Frame
    struct {
        const char sof_byte;
        const char eof_byte;
        const char delim_byte;
        const size_t max_msg_data_size;
        const size_t max_msg_frame_size;
    }protocol; // ASCII Protocol Definitions

    const struct Topic* pubMap;
    const struct Param* paramMap;

}Coms;

 

static struct Coms comsInit(const struct Topic* pubMap, const struct Param* paramMap){
    struct Coms coms = {
        .state = IDLE,
        .rxFrame = {.bufSize = 0, .cmdID = 0, .id = 0},
        .txFrame = {.bufSize = 0, .cmdID = 0, .id = 0},
        .protocol = {
            .sof_byte = SOF_BYTE,
            .eof_byte = EOF_BYTE,
            .delim_byte = DELIM_BYTE,
            .max_msg_data_size = MAX_MSG_DATA_SIZE,
            .max_msg_frame_size = MAX_MSG_FRAME_SIZE
        },
        .pubMap = pubMap,
        .paramMap = paramMap,
    };
    return coms;
}
static inline const char* comsGetPubFmt(struct Coms* coms, enum Publishers pubID){return coms->pubMap[pubID].format;}
static inline const char* comsGetPubName(struct Coms* coms, enum Publishers pubID){return coms->pubMap[pubID].name;}
static inline const char* comsGetParamFmt(struct Coms* coms, enum ParamRegisters pubID){return coms->paramMap[pubID].format;}
static inline const char* comsGetParamName(struct Coms* coms, enum ParamRegisters pubID){return coms->paramMap[pubID].name;}

static inline  char  comsEncodePubID_(enum Publishers pubID){return pubID + 'a';}
static inline  char  comsDecodePubID_(char pubID){return pubID - 'a';}
static inline  char  comsEncodeParamID_(enum ParamRegisters paramID){return paramID + 'a';}
static inline  char  comsDecodeParamID_(char paramID){return paramID - 'a';}
static inline  char  comsEncodeCmdID_(enum Commands cmdID){return cmdID + 'a';}
static inline  char  comsDecodeCmdID_(char cmdID){return cmdID - 'a';}

//****** Message Tranmission packetization  ***************//

//@Brief: Formats and Packets a Message, sends to Serial RingBuffer
static bool comsSendMsg(struct Coms* coms, struct Serial* ser, enum Publishers pubID, ...){
    const uint8_t DATA_IDX = 2;
    uint8_t* packetBuf = coms->txFrame.buf; // readability
    const char* pubFmt = comsGetPubFmt(coms, pubID);
    va_list args;
    va_start(args, pubID);
    const size_t dataSize = vsnprintf((char*)&packetBuf[DATA_IDX],MAX_MSG_DATA_SIZE, pubFmt, args); 
    va_end(args);
    if(dataSize > MAX_MSG_DATA_SIZE - 1 ){return false;}
    size_t idx = 0;
    packetBuf[idx++] = SOF_BYTE;
    packetBuf[idx++] = comsEncodePubID_(pubID);
    idx += dataSize;
    packetBuf[idx++] = EOF_BYTE;
    serialSend(ser, packetBuf, idx);
    return true;
}

//@Brief: Processes Received Data from Serial RingBuffer Into a Message Frame
//@Description: Decodes and Verifies Message Frame from Serial RingBuffer
//@Return: True if Message Frame Complete, False if Incomplete
static bool comsGrabMsg(struct Coms* coms, struct Serial* ser){
    uint8_t byte; // working byte from ring buffer
    while(rbGet(&ser->rxRB,  &byte) == true){
        switch(coms->state){
            case IDLE:
                if(byte == coms->protocol.sof_byte){
                    coms->state=CMDTYPE;
                    coms->rxFrame.bufSize = 0;
                    memset(coms->rxFrame.buf, 0, coms->protocol.max_msg_data_size);
                }
                break;
            case CMDTYPE:
                byte = comsDecodeCmdID_(byte);
                if( byte >= NUM_CMDS){coms->state=ERROR; break;} // Failed to match Command
                coms->rxFrame.cmdID = byte;
                coms->state=ID;
                break;
            case ID:
                coms->rxFrame.id = byte;
                coms->state = DATA;
                break;
            case DATA:
                if(byte == coms->protocol.sof_byte){coms->state=ERROR;break;} // Early Escape
                // Check for Buffer Overflow, receive data buffer contains only DATA
                if(coms->rxFrame.bufSize == coms->protocol.max_msg_data_size){coms->state=ERROR;break;} 
                if(byte == coms->protocol.eof_byte){
                    coms->rxFrame.buf[coms->rxFrame.bufSize++]='\0'; // null term char array!
                    coms->state=IDLE;
                    return true; // Message Frame Complete
                }
                coms->rxFrame.buf[coms->rxFrame.bufSize++]=byte;
                break;
            case ERROR:
                coms->state=IDLE;
                break;
            default:
                coms->state=IDLE;
                break;
        }
    }
    return false; // Message Frame Incomplete
}


//@Brief: Gets a Parameter Value sends to Serial
static void comsGetCmd(struct Coms *coms, struct Serial *ser){
    char valBuf[7]; // working buffer
    uint8_t paramIdx = coms->rxFrame.id;

    const char *argFmt = comsGetParamFmt(coms, paramIdx);
    snprintf(valBuf, sizeof(valBuf), argFmt, *coms->paramMap[paramIdx].param);
    comsSendMsg(coms, ser, PUB_CMD_RET, valBuf);
}

//@Brief: Sets a Parameter Value from Message Frame
static void comsSetCmd(struct Coms *coms){
    float val; // working variable
    uint8_t paramIdx = coms->rxFrame.id;
    val = atof((char*)coms->rxFrame.buf); // read in paramater value from start of data
    *coms->paramMap[paramIdx].param = val; // lookup into param Map 
}







// @Brief: Processes a Message Frame and Executes Command
static void comsProcessMsg(struct Coms * coms, struct Serial *ser){
    uint8_t paramIdx = coms->rxFrame.id;
    if(paramIdx >= NUM_PARAMS){
        comsSendMsg(coms, ser,PUB_ERROR, errorGetString(ERR_INVALID_PARAM));
        return;
    }
    if(coms->paramMap[paramIdx].param == NULL){
        comsSendMsg(coms, ser, PUB_ERROR, errorGetString(ERR_INVALID_ACCESS));
        return;
    }
    switch(coms->rxFrame.cmdID){
        case CMD_GET:
            comsGetCmd(coms, ser);    
            break;
        case CMD_SET:
            comsSetCmd(coms);
            break;
        case CMD_RUN:
            comsSendMsg(coms, ser, PUB_ERROR, errorGetString(ERR_CMD_NOT_IMPLEMENTED));
            break;
        default:
            comsSendMsg(coms, ser, PUB_ERROR, errorGetString(ERR_INVALID_CMDTYPE));
            break;
    }
}



#endif // COMS_H