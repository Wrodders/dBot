#ifndef COMS_H
#define COMS_H
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdbool.h>

#include "mcuComs.h"

/* PUB-RPC Communication Implementation *********************

*****************************************************************/

struct Topic {
    enum  Publishers id;     // Topic Register ID
    const char* format;      // Topic Register Format
    const uint8_t nArgs;     // Number of Arguments
}Topic;

struct Param {
    enum ParamRegisters id;  // Param Register ID
    const char* format;      // Param Register Format
    float *param;            // Pointer to Parameter Value
}Param;

struct RunFunc{
    enum RemoteProcedures id; // Remote Procedure ID
    const char* format;       // Argument Format
    const uint8_t nArgs;      // Number of Arguments
    const void* func;         // Function Pointer
}RunFunc;

enum COMS_DECODE_STATE{
    COMS_DECODE_IDLE = 0, 
    COMS_DECODE_CMDTYPE,
    COMS_DECODE_ID, 
    COMS_DECODE_DATA, 
    COMS_DECODE_ERROR
}COMS_DECODE_STATE; // Deserializing State Machine States


struct Coms{
    enum COMS_DECODE_STATE decodeState;
    struct CmdFrame rxFrame;    // Reception Protocol Frame
    struct TopicFrame txFrame; // Transmission Protocol Frame
    struct Protocol protocol;

    struct Topic pubMap[NUM_PUBS];
    struct Param paramMap[NUM_PARAMS];
}Coms;


static bool comsRegisterPub(struct Coms* coms, enum Publishers pubID, const char* format){
    if(pubID >= NUM_PUBS){return false;} // invalid publisher ID
    coms->pubMap[pubID].format = format;

    return true;
};


//@Brief: Registers a Parameter in the Parameter Map
//@Description: Associates a Parameter with a Parameter ID
//@Return: True if Registration Successful, False if Invalid Parameter ID
static bool comsRegisterParam(struct Coms* coms, enum ParamRegisters paramID, const char* format, float* param){
    if(paramID >= NUM_PARAMS){return false;} // invalid parameter ID
    coms->paramMap[paramID].format = format;
    coms->paramMap[paramID].param = param;
    return true;
};

static struct Coms comsInit(void){
    struct Coms coms = {
        .decodeState = COMS_DECODE_IDLE,
        .rxFrame = {.bufSize = 0, .cmdID = 0, .id = 0},
        .txFrame = {.bufSize = 0, .id = 0},
        .protocol = {
            .sof_byte = SOF_BYTE,
            .eof_byte = EOF_BYTE,
            .delim_byte = DELIM_BYTE,
            .max_msg_data_size = MAX_MSG_DATA_SIZE,
            .max_msg_frame_size = MAX_MSG_FRAME_SIZE
        },
        .pubMap = {},
        .paramMap = {}
    };
    return coms;
}

static inline const char* comsGetPubFmt(struct Coms* coms, enum Publishers pubID){return coms->pubMap[pubID].format;}
static inline const char* comsGetParamFmt(struct Coms* coms, enum ParamRegisters pubID){return coms->paramMap[pubID].format;}


//@Brief: Serializes Topic Message ASCII ProtocolTX Frame
static void comsSerializeTopicMsg(struct Coms  *const coms, const enum Publishers pubID, va_list args ){
    const uint8_t DATA_IDX = 2;
    uint8_t* packetBuf = coms->txFrame.buf; // working buffer to fill
    const char* pubFmt = comsGetPubFmt(coms, pubID); // format string
    const size_t dataSize = vsnprintf((char*)&packetBuf[DATA_IDX], MAX_MSG_DATA_SIZE, pubFmt, args);
    size_t idx = 0; // working index
    packetBuf[idx++] = coms->protocol.sof_byte; // start of frame
    packetBuf[idx++] = comsEncodeID_(pubID); // custom encode ID (avoids non-printable chars)
    idx += dataSize; // move index to end of data including null term
    packetBuf[idx++] = coms->protocol.eof_byte; // end of frame
    coms->txFrame.bufSize = idx; // update frame size
}

// @Brief: Byte by Byte Deserialization of Message Frame
// @Return: True if Message Frame Complete, False if Incomplete
// @Description: Ids and Data are stored in the RX Frame Buffer
static bool comsDeserializeCmdMsg(struct Coms* coms, uint8_t byte){
    switch(coms->decodeState){
            case COMS_DECODE_IDLE: // wait for start of frame
                if(byte == coms->protocol.sof_byte){
                    coms->decodeState=COMS_DECODE_CMDTYPE;
                    coms->rxFrame.bufSize = 0; // clear buffer new message to parse
                    memset(coms->rxFrame.buf, 0, coms->protocol.max_msg_data_size); 
                }
                break;
            case COMS_DECODE_CMDTYPE:
                byte = comsDecodeID_(byte); // custom decode ID (avoids non-printable chars)
                if( byte >= NUM_CMDS){coms->decodeState=COMS_DECODE_ERROR; break;} // Failed to match command
                coms->rxFrame.cmdID = byte; // store command ID
                coms->decodeState=COMS_DECODE_ID; // move to next state
                break;
            case COMS_DECODE_ID:
                coms->rxFrame.id = comsDecodeID_(byte); // custom decode ID (avoids non-printable chars)
                coms->decodeState = COMS_DECODE_DATA;
                break;
            case COMS_DECODE_DATA:
                if(byte == coms->protocol.sof_byte){coms->decodeState=COMS_DECODE_ERROR;break;} // Early Escape
                // Check for Buffer Overflow, receive data buffer contains only DATA
                if(coms->rxFrame.bufSize == coms->protocol.max_msg_data_size){coms->decodeState=COMS_DECODE_ERROR;break;} 
                if(byte == coms->protocol.eof_byte){
                    coms->rxFrame.buf[coms->rxFrame.bufSize++]='\0'; // null term char array!
                    coms->decodeState=COMS_DECODE_IDLE;
                    return true; // Message Frame Complete
                }
                coms->rxFrame.buf[coms->rxFrame.bufSize++]=byte;
                break;
            case COMS_DECODE_ERROR:
                coms->decodeState=COMS_DECODE_IDLE;
                break;
            default:
                coms->decodeState=COMS_DECODE_IDLE;
                break;
        }
    return false; // Message Frame Incomplete
}

#endif // COMS_H