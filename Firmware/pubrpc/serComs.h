#ifndef SER_COMS_H
#define SER_COMS_H
/************************************************************************************
 *  @file    sercoms.h
 *  @brief   serial(uart) implementation of the pub-rpc communication interface
 *  @date    2025-01-05
 *  @version 1.0.0
 *  @note    This is the MCU side of the PUB-SUB RPC communication interface.
 ************************************************************************************/

#include "../common/common.h"
#include "../drivers/serial.h"

#include "../common/errorCodes.h"

/* PUB_RPC COMS PROTOCOL MCU Application Definitions ***************
Pub-RPC Specification Protocol Frame Structure
          |-----------Packet----------|
          ******|---Frame-------|****** 
********* |-----+---+----+- - - +-----|
 Receive  | SOF |CMD| ID | DATA | EOF |
          | 1   | 1 | 1  | .... | 1   |
********* |-----+---+----+- - - +-----|
 Transmit | SOF | Pub ID | DATA | EOF | 
        | | 1   | 1      | .... | 1   |     
********* |-----+--------+------+-----|

This Is the Application Layer of the PUB-SUB RPC Communication Interface
User defines the Protocol Frame Structure and Application Specific Commands
User defines the Application Specific Topics and Parameters
*****************************************************************/


// ----------------- Protocol Frame Definitions ----------------- //
#define SOF_BYTE '<'    // hex 0x3C 
#define EOF_BYTE  '\n'  // 
#define DELIM_BYTE ':'  // hex 0x3A
#define MAX_MSG_DATA_SIZE 512
#define MSG_OVERHEAD_SIZE  4 // SOF ID LEN EOF

#define MAX_MSG_FRAME_SIZE MSG_OVERHEAD_SIZE + MAX_MSG_DATA_SIZE
#define PROT_CMD_IDX 0  // Command position in Frame
#define PROT_ID_IDX 1   // ID position in Frame

#define ID_ASCII_OFFSET 'A' // ASCII 'A' == 65 This is needed to ensure easily typeable characters for manual debugging
                            // Allows for IDs [A, Z] + [a,z] = 52 unique IDs available


//                                ±  100  .  999   /0            
#define MAX_SERIALIZED_FLOAT_SIZE (1 + 3 + 1 + 3 + 1)

#define SERIALIZED_TELEM_FMT  "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f"
#define SERIALIZED_ERROR_FMT  "%s"
#define SERIALIZED_INFO_FMT   "%s"
#define SERIALIZED_DEBUG_FMT  "%s"
#define SERIALIZED_CMDRET_FTM "%s"

// *********** Parameters **************************** //
#define PARAM_REGISTERS(X)                            \
    X(0,  P_ID,      "P_ID")                          \
    X(1,  P_MODE,    "P_MODE")                        \
    /* Motor Speed PID */                             \
    X(2,  P_LTRGT,   "P_LTRGT")                       \
    X(3,  P_LKP,     "P_LKP")                         \
    X(4,  P_LKI,     "P_LKI")                         \
    X(5,  P_RTRGT,   "P_RTRGT")                       \
    X(6,  P_RKP,     "P_RKP")                         \
    X(7,  P_RKI,     "P_RKI")                         \
    /* Balance PID */                                 \
    X(8,  P_BTRGT,   "P_BTRGT")                       \
    X(9,  P_BKP,     "P_BKP")                         \
    X(10, P_BKI,     "P_BKI")                         \
    X(11, P_BKD,     "P_BKD")                         \
    /* AngVel Offset */                               \
    X(12, P_VTRGT,   "P_VTRGT")                       \
    X(13, P_VKP,     "P_VKP")                         \
    X(14, P_VKI,     "P_VKI")                         \
    X(15, P_VKD,     "P_VKD")                         \
    X(16, P_VAPLHA,  "P_VAPLHA")                      \
    X(17, P_ATRGT,   "P_ATRGT")                       \
    X(18, P_AKP,     "P_AKP")                         \
    X(19, P_AKI,     "P_AKI")                         \
    X(20, P_AKD,     "P_AKD")                         \
    X(21, P_AALPHA,  "P_AALPHA")                      \
    /* IMU */                                         \
    X(22, P_IMU_AA, "P_IMU_AALPHA")                   \
    X(23, P_IMU_GA, "P_IMU_GALPHA")                   \
    X(24, P_IMU_KQ, "P_IMU_KAL_Q")                    \
    X(25, P_IMU_KR, "P_IMU_KAL_R")                    \
    X(26, P_IMU_KQB, "P_IMU_KAL_QB")                  \
    X(27, P_IMU_A_XOFFSET, "P_IMU_A_XOFFSET")         \
    X(28, P_IMU_A_YOFFSET, "P_IMU_A_YOFFSET")         \
    X(29, P_IMU_A_ZOFFSET, "P_IMU_A_ZOFFSET")         \
    X(30, P_IMU_MOUNT_OFFSET, "P_IMU_MOUNT_OFFSET")   \
    X(31, P_LINVEL_FILT_A, "P_LINVEL_FILT_A")         \
    X(32, P_ANGVEL_FILT_A, "P_ANGVEL_FILT_A")         \
    X(33, NUM_PARAMS,   "NUM_PARAMS")                 \
    
  
#define PARAM_ENUM(ID, NAME, MSG) NAME = ID,
#define PARAM_STRING(ID, NAME, MSG) case NAME: return MSG;

enum ParamRegisters {
    PARAM_REGISTERS(PARAM_ENUM)
};

static const char* paramRegisterString(enum ParamRegisters param) {
    switch (param) {
        PARAM_REGISTERS(PARAM_STRING)
        default: return "UNKNOWN PARAM";
    }
}

// *********** PUBLISHERS **************************** //
#define PUBLISHERS(X)                       \
    X(0,    PUB_CMD_RET,    "CMD_RET")      \
    X(1,    PUB_ERROR,      "ERROR")        \
    X(2,    PUB_INFO,       "INFO")         \
    X(3,    PUB_DEBUG,      "DEBUG")        \
    X(4,    PUB_TELEM,      "TELEM")        \
    X(5,    NUM_PUBS,       "NUM_PUBS")     \

#define PUBLISHER_ENUM(ID, NAME, MSG) NAME = ID,
#define PUBLISHER_STRING(ID, NAME, MSG) case NAME: return MSG;

enum Publishers{
    PUBLISHERS(PUBLISHER_ENUM)
};

static const char* publisherString(enum Publishers pub){
    switch(pub){
        PUBLISHERS(PUBLISHER_STRING)
        default: return "UNKNOWN PUBLISHER";
    }
}

// TELEMETRY ARGUMENTS SERIALIZATION HELPERS
#define TELEMETRY_VARS(X)                               \
    X(0,   T_PITCH,              "PITCH")               \
    X(1,   T_ROLL,               "ROLL")                \
    X(2,   T_ACCEL_X,            "ACCEL_X")             \
    X(3,   T_ACCEL_Y,            "ACCEL_Y")             \
    X(4,   T_ACCEL_Z,            "ACCEL_Z")             \
    X(5,   T_GYRO_X,             "GYRO_X")              \
    X(6,   T_GYRO_Y,             "GYRO_Y")              \
    X(7,   T_GYRO_Z,             "GYRO_Z")              \
    X(8,   T_LEFT_SHAFTS_RPS,    "LEFT_SHAFT_RPS")      \
    X(9,   T_LEFT_WHEEL_TRGT,    "LEFT_WHEEL_TRGT")     \
    X(10,  T_LEFT_VOLTAGE,       "LEFT_VOLTAGE")        \
    X(11,  T_RIGHT_SHAFTS_RPS,   "RIGHT_SHAFT_RPS")     \
    X(12,  T_RIGHT_WHEEL_TRGT,   "RIGHT_WHEEL_TRGT")    \
    X(13,  T_RIGHT_VOLTAGE,      "RIGHT_VOLTAGE")       \
    X(14,  T_LINEAR_VEL,         "LINEAR_VEL")          \
    X(15,  T_LIN_VEL_TRGT,       "LIN_VEL_TRGT")        \
    X(16,  T_BALANCE_ANGLE,      "BALANCE_ANGLE")       \
    X(17,  T_ANGULAR_VEL,        "ANGULAR_VEL")         \
    X(18,  T_ANG_VEL_TRGT,       "ANG_VEL_TRGT")        \
    X(19,  T_STEER_DIFF,         "STEER_DIFF")          \
    X(20,  T_NUM_TELEM_VARS,     "NUM__TELEM_VARS")     \

#define TELEMETRY_ENUM(ID, NAME, MSG) NAME = ID,
#define TELEMETRY_STRING(ID, NAME, MSG) case NAME: return MSG;

enum TelemetryVars{
    TELEMETRY_VARS(TELEMETRY_ENUM)
};

static const char* telemetryVarString(enum TelemetryVars var){
    switch(var){
        TELEMETRY_VARS(TELEMETRY_STRING)
        default: return "UNKNOWN TELEMETRY VAR";
    }
}


/* PUB-RPC Serial Communication Implementation **************************************************** */
struct Protocol {
    const char sof_byte;
    const char eof_byte;
    const char delim_byte;
    const size_t max_msg_data_size;
    const size_t max_msg_frame_size;
}; // ASCII  Protocol 

// ----------------- Remote Procedure Definitions ----------------- //
enum RemoteProcedures{
    CMD_GET = 0, // Get Parameter Value
    CMD_SET,     // Set Parameter Value
    NUM_RPC
}; // Command Type 

struct CmdFrame{
    size_t bufSize; // current size of msg buffer
    enum RemoteProcedures cmdID; // Command Type identifier
    uint8_t id;  // Register id for map  
    uint8_t buf[MAX_MSG_FRAME_SIZE]; 
}; 

struct TopicFrame{
    size_t bufSize; // current size of msg buffer
    uint8_t id;  // Register id for map  
    uint8_t buf[MAX_MSG_FRAME_SIZE];
};

struct Topic {
    enum  Publishers id;     // Topic Register ID
    const char* format;      // Topic Register Format
    const uint8_t nArgs;     // Number of Arguments
};

struct Param {
    enum ParamRegisters id;  // Param Register ID
    const char* format;      // Param Register Format
    float *reg;            // Pointer to Parameter Value
};

struct RPC{
    enum RemoteProcedures id; // Remote Procedure ID
    void (*func)(struct Serial*, struct CmdFrame*, struct Param* const, char *retBuf); // Fixed function pointer
};

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
    char cmdret[MAX_SERIALIZED_FLOAT_SIZE]; // Command Return Value serialized Buffer

    struct Topic pubMap[NUM_PUBS];
    struct Param paramMap[NUM_PARAMS];
    struct RPC rpcMap[NUM_RPC];
};

static inline char  comsEncodeID_(uint8_t id){return id + ID_ASCII_OFFSET;}
static inline uint8_t  comsDecodeID_(char id){return id - ID_ASCII_OFFSET;}
static inline const char* comsGetPubFmt(struct Coms* coms, enum Publishers pubID){return coms->pubMap[pubID].format;}
static inline const char* comsGetParamFmt(struct Coms* coms, enum ParamRegisters pubID){return coms->paramMap[pubID].format;}

// Core RPC Functions Prototypes
static void _comsExecGetCmd(struct Serial *ser, struct CmdFrame *rxFrame, struct Param * const param, char* retBuf);
static void _comsExecSetCmd(struct Serial *ser, struct CmdFrame *rxFrame, struct Param * const param, char* retBuf);

//@Brief: Registers a Publisher in the Publisher Map
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
    coms->paramMap[paramID].reg = param;
    return true;
};
//@Brief: Registers a Remote Procedure in the RPC Map
static bool comsRegisterRPC(struct Coms* coms, enum RemoteProcedures rpcID, void (*func)(struct Serial*, struct CmdFrame*, struct Param* const, char *retBuf)){
    if(rpcID >= NUM_RPC){ return false; } // invalid RPC ID
    coms->rpcMap[rpcID].func = func;
    return true;
}

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
        .cmdret = {0},
        .pubMap = {},
        .paramMap = {},
        .rpcMap = {}

    };
    // Register core RPC functions
    comsRegisterRPC(&coms, CMD_GET, _comsExecGetCmd);
    comsRegisterRPC(&coms, CMD_SET, _comsExecSetCmd);
    return coms;
}

//@Brief: Formats and Packets a Message, sends to Serial RingBuffer
static bool comsSendMsg(struct Coms* coms, struct Serial* ser, enum Publishers pubID, ...) {
    va_list args;
    va_start(args, pubID);
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
    va_end(args);
    serialSend(ser, coms->txFrame.buf, coms->txFrame.bufSize);
    return true;
}

//@Brief: Processes Received Data from Serial RingBuffer Into a Message Frame
//@Description: Decodes and Verifies Message Frame from Serial RingBuffer
//@Return: True if A Message Frame Complete, False if Incomplete
//@Note: Only Grabs One Message Frame at a Time, 
//       ring buffer ensures that if multiple or partial frames are available 
//       they will processed in subsequent calls  
// Validates inplace.
static bool comsGrabCmdMsg(struct Coms* coms, struct Serial* ser){
    uint8_t byte; // working byte from ring buffer
    while(rbGet(&ser->rxRB,  &byte) == true){ // iterate over bytes available
       switch(coms->decodeState){
            case COMS_DECODE_IDLE: // wait for start of frame
                if(byte == coms->protocol.sof_byte){
                    coms->decodeState=COMS_DECODE_CMDTYPE;
                    coms->rxFrame.bufSize = 0; // clear buffer new message to parse
                    memset(coms->rxFrame.buf, 0, coms->protocol.max_msg_data_size); 
                }
                break;
            case COMS_DECODE_CMDTYPE:
                byte = comsDecodeID_(byte); // asccii offset decode ID (avoids non-typeable chars)
                if( byte >= NUM_RPC){coms->decodeState=COMS_DECODE_ERROR; break;} // Failed to match command
                coms->rxFrame.cmdID = byte; // store rpc command ID
                coms->decodeState=COMS_DECODE_ID; // move to next state
                break;
            case COMS_DECODE_ID:
                byte = comsDecodeID_(byte); 
                if( byte >= NUM_PARAMS){coms->decodeState=COMS_DECODE_ERROR; break;} // Failed to match parameter
                coms->rxFrame.id = byte; // store decoded parameter ID
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
                coms->rxFrame.bufSize = 0; // clear buffer new message to parse
                memset(coms->rxFrame.buf, 0, coms->protocol.max_msg_data_size); 
                break;
            default:
                coms->decodeState=COMS_DECODE_IDLE;
                break;
        }
    }
    return false; // Message Frame Incomplete
}

static void comsExecuteRPC(struct Coms* coms, struct Serial* ser){
    uint8_t cmdID = coms->rxFrame.cmdID;
    uint8_t paramIdx = coms->rxFrame.id;
    coms->rpcMap[cmdID].func(ser, &coms->rxFrame, &coms->paramMap[paramIdx], coms->cmdret); // Call function pointer
    if(coms->cmdret[0] != '\0'){
        comsSendMsg(coms, ser, PUB_CMD_RET, coms->cmdret); // return value back to serial
        memset(coms->cmdret, 0, sizeof(coms->cmdret)); // clear return buffer
    }
}
   
// *********** Core RPC Functions **************************** //
//@Brief: Returns a Parameters Value over CMD_RET Message
//@Description: Looks up Parameter Value, encodes to string as per parameter format
static void _comsExecGetCmd(struct Serial *ser, struct CmdFrame *rxFrame, struct Param * const param, char* retBuf){
    uint8_t paramIdx = rxFrame->id;
    const char *argFmt = param->format; // get format string
    // Serialize Parameter Value to buffer using its format string
    snprintf(retBuf, MAX_SERIALIZED_FLOAT_SIZE, argFmt, *param->reg);
}

//@Brief: Sets a Parameter Value in the Parameter Map
static void _comsExecSetCmd(struct Serial *ser, struct CmdFrame *rxFrame, struct Param * const param, char* retBuf){
    float val; // working variable
    uint8_t paramIdx = rxFrame->id;
    val = atof((char*)rxFrame->buf); // read in paramater value from start of data
    *param->reg = val; // lookup into param Map 
}
#endif // SER_COMS_H