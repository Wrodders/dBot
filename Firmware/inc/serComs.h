#ifndef SER_COMS_H
#define SER_COMS_H

#include "../common/common.h"
#include "../drivers/serial.h"

#include "../common/errorCodes.h"
#include "../pubrpc/coms.h"

/* PUB-RPC Serial Communication Interface ****************************************************
This is the MCU side of the PUB-SUB RPC communication interface.
The MCU Publishes data under Topics usign Message Topic Frame Protocol
The MCU Receives RPC Commands of type GET, SET, RUN using Cmd Frame Protocol
The MCU defines a set of Parameters that can be accessed by the RPC Commands
The MCU sends the retrun value of the RPC command over the PUB_CMD_RET Topic
*********************************************************************************************/

//@Brief: Formats and Packets a Message, sends to Serial RingBuffer
static bool comsSendMsg(struct Coms* coms, struct Serial* ser, enum Publishers pubID, ...) {
    va_list args;
    va_start(args, pubID);
    comsSerializeTopicMsg(coms, pubID, args); // serialize message into packet implementation
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
static bool comsGrabCmdMsg(struct Coms* coms, struct Serial* ser){
    uint8_t byte; // working byte from ring buffer
    while(rbGet(&ser->rxRB,  &byte) == true){ // iterate over bytes available
        if(comsDeserializeCmdMsg(coms, byte)){return true;} // Message Frame Complete
    }
    return false; // Message Frame Incomplete
}

//@Brief: Returns a Parameters Value over CMD_RET Message
//@Description: Looks up Parameter Value, encodes to string as per parameter format
static void comsExecGetCmd(struct Coms *coms, struct Serial *ser){
    char valBuf[MAX_SERIALIZED_FLOAT_SIZE]; // working buffer
    uint8_t paramIdx = coms->rxFrame.id;
    const char *argFmt = comsGetParamFmt(coms, paramIdx);
    // Serialize Parameter Value to buffer using its format string
    snprintf(valBuf, sizeof(valBuf), argFmt, *coms->paramMap[paramIdx].param);
    comsSendMsg(coms, ser, PUB_CMD_RET, valBuf); // return value back to serial
}

//@Brief: Sets a Parameter Value in the Parameter Map
static void comsExecSetCmd(struct Coms *coms){
    float val; // working variable
    uint8_t paramIdx = coms->rxFrame.id;
    val = atof((char*)coms->rxFrame.buf); // read in paramater value from start of data
    *coms->paramMap[paramIdx].param = val; // lookup into param Map 
}

//@Brief: Processes a Received Command Message Frame and Executes Command
static void comsProcessCmdMsg(struct Coms * coms, struct Serial *ser){
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
            comsExecGetCmd(coms, ser);    
            break;
        case CMD_SET:
            comsExecSetCmd(coms);
            break;
        case CMD_RUN:
            comsSendMsg(coms, ser, PUB_ERROR, errorGetString(ERR_CMD_NOT_IMPLEMENTED));
            break;
        default:
            comsSendMsg(coms, ser, PUB_ERROR, errorGetString(ERR_INVALID_CMDTYPE));
            break;
    }
}
#endif // SER_COMS_H