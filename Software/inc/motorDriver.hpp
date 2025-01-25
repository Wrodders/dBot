#ifndef MOTOR_CNTRL_H
#define MOTOR_CNTRL_H

// Motor Controller Driver Functionality
#include "../common/common.h"

#include <queue>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <unistd.h>
#include <string.h>
#include <sys/fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/select.h>
#include <thread>

enum CMD_TYPE{
    CMD_GET = 0,
    CMD_SET,
    CMD_RUN,
    CMD_NUMS
};

struct CmdMsg{
    uint8_t id;     // Param ID
    enum CMD_TYPE type;   // cmd type 
    float data;
    time_t timestamp;
};

struct CmdRet{
    uint8_t id; // Param ID
    float data;
    time_t timestamp;
};

struct LogMsg{
    uint8_t id; // PubID
    std::string message;
    time_t timestamp;
};

struct MotorState{
    struct {
        float pitch, roll;
    } imu;
    struct {
        float lw, rw;
        float linVel, angVel;
        float xPos, yPos, theta;
    } odom;
    struct {
        float targetLeftWheelRPM, targetRightWheelRPM;
        float voltageLeft, voltageRight;
        float outputBalance; 
        float angularVelOffset;
        float targetAngle;
        float targetLinVel, targetAngVel;
    } cntrl;
    time_t timestamp;
};

class MotorDriver{
    public:
        MotorDriver(const char *port){
            this->fd = configSerialPort(port);
            if(this->fd == -1){
                throw std::runtime_error("Unable to configure serial port");
            }
            readThread = std::thread(&MotorDriver::writeSerialWorker, this);
            writeThread = std::thread(&MotorDriver::writeSerialWorker, this);
        }
        ~MotorDriver(){
            close(this->fd); // Close serial port
            this->readThread.join(); 
            this->writeThread.join();
        }
        // Thread safe to get the motor state from the queue
        inline MotorState *getMotorState(){return &this->motorState;}

        inline void pushSetCmd(uint8_t id, float data){cmdQueue.push({id, CMD_SET, data, time(NULL)});}

        inline void pushGetCmd(uint8_t id){cmdQueue.push({id, CMD_GET, 0, time(NULL)});}


    private:
        int fd; // serial port
        std::thread readThread, writeThread;
        std::map<std::string, struct ParamInfo> paramLookup;
        struct MotorState motorState;
        std::queue<struct CmdMsg> cmdQueue;
        std::queue<struct MotorState> stateQueue;
        std::queue<struct LogMsg> logQueue;    
        enum PUB_ID{PUB_CMD_RET = 0,PUB_ERROR,PUB_INFO,PUB_DEBUG, PUB_STATE, PUB_NUMS}; 

        int processMsg(char *buffer){
            if (buffer[0] == '<') {  // Check for start of message
                uint8_t id = buffer[1]-'a';
                switch(id){
                    case PUB_STATE:
                        sscanf(&buffer[2], 
                        "%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f\n",
                        &motorState.imu.pitch, &motorState.imu.roll,
                        &motorState.odom.lw, &motorState.odom.rw,
                        &motorState.odom.linVel, &motorState.odom.angVel,
                        &motorState.odom.xPos, &motorState.odom.yPos, &motorState.odom.theta,
                        &motorState.cntrl.targetLeftWheelRPM, &motorState.cntrl.targetRightWheelRPM,
                        &motorState.cntrl.ul, &motorState.cntrl.ur, 
                        &motorState.cntrl.ub, &motorState.cntrl.ua, &motorState.cntrl.uv);
                        motorState.timestamp = time(NULL);
                        break;   
                        // write to log queue
                    case PUB_INFO || PUB_ERROR || PUB_DEBUG:
                        logQueue.push({id, &buffer[2], time(NULL)});
                        break;
                    case PUB_CMD_RET:
                        break;                        
                    default:
                        printf("Received message: %s", buffer);
                        break;
                }
                
            }
        }



        void writeSerialWorker() {
            FILE *fp = fdopen(fd, "r");  // Open file stream for reading
            if (fp == NULL) {
                perror("Failed to associate file stream");
                return;
            }
            char buffer[256];
            while (true) {
            if (fgets(buffer, sizeof(buffer), fp) != NULL) {
                processMsg(buffer);
            } else {
                std::cerr << "Error reading from serial port" << std::endl;
                break;
            }
            }
        }

        void writeSerialWorker(){
            char buffer[64];
            while(!stop){
                if(!cmdQueue.empty()){
                    struct CmdMsg cmd = cmdQueue.front();
                    sprintf(buffer, "<%c:%f>\n", cmd.id+'a', cmd.data);
                    write(this->fd, buffer, strlen(buffer));
                    cmdQueue.pop();
                }
            }
        }

        int configSerialPort(const char *port) {
            struct termios options;
            int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
            if (fd == -1) {
                perror("Unable to open serial port");
                throw std::runtime_error("Unable to open serial port");
            }
            // Get current options
            if (tcgetattr(fd, &options) < 0) {
                perror("Error getting serial port attributes");
                close(fd);
                return -1;
            }
            // Set baud rate (example: 9600)
            cfsetispeed(&options, B115200);    // Input baud rate
            cfsetospeed(&options, B115200);    // Output baud rate
            // Set 8 data bits, no parity, 1 stop bit
            options.c_cflag &= ~PARENB;      // No parity
            options.c_cflag &= ~CSTOPB;      // 1 stop bit
            options.c_cflag &= ~CSIZE;       // Mask data bits
            options.c_cflag |= CS8;          // 8 data bits
            // Set canonical mode (line buffering)
            options.c_lflag |= ICANON;       // Enable canonical mode
            options.c_lflag |= ECHO;         // Enable echo
            options.c_lflag |= ECHOE;        // Enable erasure
            options.c_lflag |= ISIG;         // Enable signal processing (Ctrl+C, Ctrl+Z, etc.)
            // Disable non-canonical modes like raw input or extended input processing
            options.c_lflag &= ~(ICRNL | INLCR); // No newline translation
            // Set timeout (for example, 5 seconds)
            options.c_cc[VMIN] = 1;          // Minimum number of characters for non-blocking read
            options.c_cc[VTIME] = 50;        // Timeout in deciseconds (500ms)
            // Apply the settings to the serial port
            if (tcsetattr(fd, TCSANOW, &options) < 0) {
                std::cerr << "Error setting serial port attributes" << std::endl;
                close(fd);
                return -1;
            }
            return fd;
        }

};







#endif  // ßMOTOR_CNTRL_Hß