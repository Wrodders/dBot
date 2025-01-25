#ifndef MCU_COMS_H
#define MCU_COMS_H

#include <stdint.h>
#include <stddef.h>

// ***** ASCII PROTOCOL DEFINITIONS ************ //
/*
          |-----------Packet----------|
          ******|---Frame-------|****** 
********* |-----+---+----+- - - +-----|
 Receive  | SOF |CMD| ID | DATA | EOF |
          | 1   | 1 | 1  | .... | 1   |
********* |-----+---+----+- - - +-----|
 Transmit | SOF | Pub ID | DATA | EOF | 
        | | 1   | 1      | .... | 1   |     
********* |-----+--------+------+-----|
               
SOF -- Start of Packet Frame
CMD -- CMD Type Get Set Run
ID --  Topic Identifier 
DATA -- LEN bytes of data
EOF - End of Packet Frame 
*/

#define SOF_BYTE '<' // hex 0x3C 
#define EOF_BYTE  '\n' // 
#define DELIM_BYTE ':' // hex 0x3A
#define MAX_MSG_DATA_SIZE 512
#define MSG_OVERHEAD_SIZE  4 // SOF ID LEN EOF

#define MAX_MSG_FRAME_SIZE MSG_OVERHEAD_SIZE + MAX_MSG_DATA_SIZE
#define PROT_CMD_IDX 0  // Command position in Frame
#define PROT_ID_IDX 1   // ID position in Frame

enum Commands{
    CMD_GET = 0, // Get Parameter Value
    CMD_SET,     // Set Parameter Value
    CMD_RUN,     // Remote Execute Function
    NUM_CMDS
}Commands; // Command Type

enum Publishers {
    PUB_CMD_RET = 0,
    PUB_ERROR,
    PUB_INFO, 
    PUB_DEBUG, 
    // ADD Application Specific Publishers
    PUB_STATE,
    NUM_PUBS
}Publishers; // Publish Topic IDs

enum ParamRegisters{
    P_ID = 0,
    P_MODE,
    // Motor Speed PID 
    P_LTRGT, P_LKP, P_LKI,
    P_RTRGT, P_RKP, P_RKI,
    // Balance PID
    P_BTRGT,
    P_BKP,
    P_BKI,
    P_BKD,
    // AngVel Offset
    P_VTRGT,
    P_VKP,
    P_VKI,
    P_VKD,
    P_VAPLHA,

    P_ATRGT,
    P_AKP,
    P_AKI,
    P_AKD,
    P_AALPHA,

    NUM_PARAMS
}ParamRegisters; // Parameter Ids

enum RemoteProcedures {
    RUN_RESET = 0,
    RUN_UPGRADE,
    RUN_CALIBRATE,

    NUM_RUN
}RemoteProcedures; // Run Command Id

#endif // MCU_COMS_H