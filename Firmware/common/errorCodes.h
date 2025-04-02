#ifndef ERROR_CODE_H
#define ERROR_CODE_H

#define ERROR_CODES(X)  \
    X(0, ERR_NONE, "") \
    X(1, ERR_TIMEOUT, "Timeout Error") \
    X(2, ERR_INVALID_PARAM, "Invalid Parameter") \
    X(3, ERR_INVALID_ACCESS, "Invalid Access") \
    X(4, ERR_CMD_NOT_IMPLEMENTED, "Command Not Implemented") \
    X(5, ERR_INVALID_CMDTYPE, "Invalid Command Type") \


#define ERROR_ENUM(ID, NAME, MSG) NAME = ID,
#define ERROR_STRING(ID, NAME, MSG) case NAME: return MSG;

enum ErrorCode{
    ERROR_CODES(ERROR_ENUM)
};

static const char* errorGetString(enum ErrorCode code){
    switch(code){
        ERROR_CODES(ERROR_STRING)
        default: return "UNKNOWN ERROR";
    }
}


#endif // ERROR_CODE_H