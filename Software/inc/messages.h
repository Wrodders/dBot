#ifndef MESSAGES_H
#define MESSAGES_H


struct MOTOR_STATE {
    struct{
        float wL; wR; 
        float linVel; angVel;
    }odom;
    struct{
        float refL; refR;
        float uL; uR;
        float uBal;
        float uAng;
        float uVel;
    }ctrl;
    struct{
        float roll; pitch;
    }imu;
    float cmdRet;
};

struct CMD_MSG{
    char address[20];
    char type;
    char id;
    float value;
}

struct 


#endif // MESSAGES_H