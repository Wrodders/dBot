#ifndef MCU_COMS_H
#define MCU_COMS_H

#include <stdint.h>
#include <stddef.h>

/* PUB_RPC Communication MCU Application Definitions ***************
Protocol Frame Structure(s)
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
Defines the Protocol Frame Structure and Application Specific Commands
Defines the Application Specific Topics and Parameters

*****************************************************************/

#define SOF_BYTE '<' // hex 0x3C 
#define EOF_BYTE  '\n' // 
#define DELIM_BYTE ':' // hex 0x3A
#define MAX_MSG_DATA_SIZE 512
#define MSG_OVERHEAD_SIZE  4 // SOF ID LEN EOF

#define MAX_MSG_FRAME_SIZE MSG_OVERHEAD_SIZE + MAX_MSG_DATA_SIZE
#define PROT_CMD_IDX 0  // Command position in Frame
#define PROT_ID_IDX 1   // ID position in Frame

#define ID_ASCII_OFFSET 'A' // ASCII 'A' == 65 This is needed to avoid non-printable characters
                            // Allows for IDs [A, Z] + [a,z] = 52 unique IDs

struct Protocol {
    const char sof_byte;
    const char eof_byte;
    const char delim_byte;
    const size_t max_msg_data_size;
    const size_t max_msg_frame_size;
}; // ASCII  Protocol 

struct CmdFrame{
    size_t bufSize; // current size of msg buffer
    uint8_t cmdID; // Command Type identifier
    uint8_t id;  // Register id for map  
    uint8_t buf[MAX_MSG_FRAME_SIZE]; 
}CmdFrame; 

struct TopicFrame{
    size_t bufSize; // current size of msg buffer
    uint8_t id;  // Register id for map  
    uint8_t buf[MAX_MSG_FRAME_SIZE];
}TopicFrame;

enum Commands{
    CMD_GET = 0, // Get Parameter Value
    CMD_SET,     // Set Parameter Value
    CMD_RUN,     // Remote Execute Function
    NUM_CMDS
}; // Command Type 

static inline  char  comsEncodeID_(uint8_t id){return id + ID_ASCII_OFFSET;}
static inline  char  comsDecodeID_(char id){return id - ID_ASCII_OFFSET;}

//                                ±  100  .  999   /0            
#define MAX_SERIALIZED_FLOAT_SIZE (1 + 3 + 1 + 3 + 1)

#define SERIALIZED_IMU_FMT    "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f"
#define SERIALIZED_STATE_FMT  "%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f:%0.3f"
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
    X(30, NUM_PARAMS,   "NUM_PARAMS")                 \
  
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

// *********** REMOTE PROCEDURES **************************** //

#define REMOTE_PROCEDURES(X)                            \
    X(0,    RUN_RESET,      "RESET")                    \
    X(1,    RUN_CALIB,      "CALIB")                    \
    X(2,    NUM_RPC,        "NUM_RPC")                  \

#define REMOTE_PROCEDURE_ENUM(ID, NAME, MSG) NAME = ID,
#define REMOTE_PROCEDURE_STRING(ID, NAME, MSG) case NAME: return MSG;

enum RemoteProcedures{
    REMOTE_PROCEDURES(REMOTE_PROCEDURE_ENUM)
};

static const char* remoteProcedureString(enum RemoteProcedures rpc){
    switch(rpc){
        REMOTE_PROCEDURES(REMOTE_PROCEDURE_STRING)
        default: return "UNKNOWN REMOTE PROCEDURE";
    }
}

// *********** PUBLISHERS **************************** //
#define PUBLISHERS(X)                       \
    X(0,    PUB_CMD_RET,    "CMD_RET")      \
    X(1,    PUB_ERROR,      "ERROR")        \
    X(2,    PUB_INFO,       "INFO")         \
    X(3,    PUB_DEBUG,      "DEBUG")        \
    X(4,    PUB_STATE,      "STATE")        \
    X(5,    PUB_IMU,        "IMU")          \
    X(6,    NUM_PUBS,       "NUM_PUBS")     \

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


// *********** Telemtry DATA STRUCTURES **************************** //
struct TelemState{
    float *leftShaftRPS,        *rightShaftRPS;
    float *leftWheelTargetRPS,  *rightWheelTargetRPS;
    float *voltageLeft,         *voltageRight;

    float *linearVelocity,      *targetLinVel,      *balanceAngle;
    float *angularVelocity,     *targetAngularVel,  *steerDiff;
}; // State data publish structure

struct TelemImu{
    float *pitch, *roll;
    float *gyroX, *gyroY, *gyroZ;
    float *accX, *accY, *accZ;
}; // Imu data publish structure

// TELEMETRY ARGUMENTS SERIALIZATION HELPERS
#define TELEMETRY_VARS(X)                              \
    X(1,   T_PITCH,                "PITCH")            \
    X(2,   T_ROLL,                 "ROLL")             \
    X(3,   T_ACCEL_X,              "ACCEL_X")          \
    X(4,   T_ACCEL_Y,              "ACCEL_Y")          \
    X(5,   T_ACCEL_Z,              "ACCEL_Z")          \
    X(6,   T_GYRO_X,              "GYRO_X")            \
    X(7,   T_GYRO_Y,              "GYRO_Y")            \
    X(8,   T_GYRO_Z,              "GYRO_Z")            \
    X(9,   T_LEFT_SHAFTS_RPS,    "LEFT_SHAFT_RPS")     \
    X(10,  T_LEFT_WHEEL_TRGT,    "LEFT_WHEEL_TRGT")    \
    X(11,  T_LEFT_VOLTAGE,       "LEFT_VOLTAGE")       \
    X(12,  T_RIGHT_SHAFTS_RPS,   "RIGHT_SHAFT_RPS")    \
    X(13,  T_RIGHT_WHEEL_TRGT,   "RIGHT_WHEEL_TRGT")   \
    X(14,  T_RIGHT_VOLTAGE,      "RIGHT_VOLTAGE")      \
    X(15,  T_LINEAR_VEL,         "LINEAR_VEL")         \
    X(16,  T_LIN_VEL_TRGT,       "LIN_VEL_TRGT")       \
    X(17,  T_BALANCE_ANGLE,      "BALANCE_ANGLE")      \
    X(18,  T_ANGULAR_VEL,        "ANGULAR_VEL")        \
    X(19,  T_ANG_VEL_TRGT,       "ANG_VEL_TRGT")       \
    X(20,  T_STEER_DIFF,         "STEER_DIFF")         \
    X(21,  T_NUM_SATE_VARS,      "NUM__STATE_VARS")    \

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
#endif // MCU_COMS_H