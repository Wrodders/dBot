#include <sys/select.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <chrono>
#include <ctime>
#include <cstring>
#include <iostream>
#include <zmq.hpp>
#include <iomanip>
#include <getopt.h>
#include <vector>
#include <queue>


#ifdef __linux__
#include "../inc/mcuComs.h"
#elif __APPLE__
#include "../../Firmware/pubrpc/mcuComs.h"
#include "../modules/cUtils/ringbuffer.h"
#endif



#define DEFAULT_PORT "/dev/serial0"
#define DEFAULT_BAUD 115200

// Debugging mode
const bool DEBUG_MODE = false;

/*
********* |-----+---+----+- - - +-----|
          | SOF | Pub ID | DATA | EOF | 
          | 1   | 1      | .... | 1   |     
********* |-----+--------+------+-----|
*/

class TWSBDriver {
    struct Telemetry{
        std::vector<float> stateVars;
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
    }telemetry_;

    struct LogEntry {
        std::string message;
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
    } logentry_;
    struct CmdRet {
        std::string msg;
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
    } cmdret_;

    

public:
    int serialFd; 
    // Constructor
    TWSBDriver( const std::string& port, int baud) {
        serialFd = configSerialPort(port, baud);
        if (serialFd == -1) {
            std::cerr << "Error: Unable to configure serial port" << std::endl;
            return;
        }
    }
    ~TWSBDriver() {
        close(serialFd);
    }

    float getTelemetryValue(enum StateVars var) {
        return telemetry_.stateVars[var];
    }

    void printTelemetryTable() {
        double timeDiff = calcSampleTimeDiff();
        std::cout << "\033[1;34m" << "Telemetry Table" << "\033[0m" << std::endl;
        std::cout << "\033[1;36m" << "+--------------------+-------------------+" << "\033[0m" << std::endl;
        std::cout << "\033[1;36m" << "| State Variable     | Value (" << std::setw(6) << std::setprecision(2) << timeDiff << " ms) |" << "\033[0m" << std::endl;
        std::cout << "\033[1;36m" << "+--------------------+-------------------+" << "\033[0m" << std::setprecision(5)<<std::endl;
        for (int i = 0; i < T_NUM_VARS; i++) {
            std::cout << "| " << std::left << std::setw(18) << stateVarString((enum StateVars)i)
                    << " | " << std::setw(17) << std::right <<getTelemetryValue((enum StateVars)i) << " |" << std::endl;
        }
        std::cout << "\033[1;36m" << "+--------------------+-------------------+" << "\033[0m" << std::endl;
        std::cout << "\033[1;31m" << "\nLo: " << "\033[0m" << std::endl;
        std::cout << "| " << std::left << std::setw(18) << "Msg: " << logentry_.message << " |" << std::endl;
        std::cout << "| " << std::setw(12) << "Timestamp: " << formatTimestamp(logentry_.timestamp) << " |" << std::endl;
        // Display the command return in a more compact format with left alignment
        std::cout << "\033[1;32m" << "\nCommand Return: " << "\033[0m" << std::endl;
        std::cout << "| " << std::left << std::setw(18) << "Msg: " << cmdret_.msg << " |" << std::endl;
        std::cout << "| " << std::setw(12) << "Timestamp: " << formatTimestamp(cmdret_.timestamp) << " |" << std::endl;
    }

    void update() {
        grabMsgFrame();
        deserializeMsg();
    }
private:

    // ***************/ Serial Port and ZMQ IO **************************/
    struct termios options;
    uint8_t rxBuffer[1024] = {0}; // Serial port read buffer
    
    // TimeStamp
    std::chrono::time_point<std::chrono::high_resolution_clock> lastTime;
    
    struct TopicMsgFrame { // Serialized Message Frame
        enum Publishers pubID; // Publisher ID
        std::string data; // ASCII Protocol Payload
    } rxMsgFrame;
    
    std::queue<TopicMsgFrame> topicMsgQueue; // Received Topic Message Queue
    //************************ PROTOCOL ***********************************/
    struct Protocol protocol = {
        .sof_byte = SOF_BYTE,
        .eof_byte = EOF_BYTE,
        .delim_byte = DELIM_BYTE,
        .max_msg_data_size = MAX_MSG_DATA_SIZE,
        .max_msg_frame_size = MAX_MSG_FRAME_SIZE
    }; // ASCII Protocol Used for Communication over Serial Port
    enum COMS_DECODE_STATE {
        COMS_DECODE_IDLE = 0, 
        COMS_DECODE_ID, 
        COMS_DECODE_DATA, 
        COMS_DECODE_ERROR
    } comsDecodeState = COMS_DECODE_IDLE;
    
    //@Brief: Grab the ASCII message frame from the serial port read buffer
    //@Description: Bytes are read from serial port and decoded into a message frame
    //@Note: If multiple complete frames are available, they are pushed to the queue in sequence
    //@Note: If partial frames are available, they can be processed in the next iteration
    void grabMsgFrame() {
        ssize_t bytesRead = read(serialFd, rxBuffer, sizeof(rxBuffer));
       
        uint8_t idx = 0;
        while ((bytesRead - idx) > 0) { // iterate through the read data
            decodeTopicMsgFrame(rxBuffer[idx++]);
        }
    }
    
    //@Brief: Decodes bytes into a message frame, pushing it to the queue when complete
    void decodeTopicMsgFrame(const uint8_t byte) {
        switch(comsDecodeState){
            case COMS_DECODE_IDLE: // Waiting for Start of Frame Byte
                if (byte == protocol.sof_byte) {
                    rxMsgFrame.data.clear(); // Clear the data buffer
                    comsDecodeState = COMS_DECODE_ID;
                }
                break;
            case COMS_DECODE_ID: // Waiting for ID Byte
                rxMsgFrame.pubID = (enum Publishers)comsDecodeID_(byte); // Grab the ID
                comsDecodeState = COMS_DECODE_DATA;
                break;
            case COMS_DECODE_DATA: // grab Data Bytes
                if(byte == protocol.sof_byte){ // Error: escape character
                    comsDecodeState = COMS_DECODE_ERROR;
                } 
                else if(byte == protocol.eof_byte){ // Message Frame Complete
                    // Add the message frame to the queue
                   
                    topicMsgQueue.push(rxMsgFrame);
                    comsDecodeState = COMS_DECODE_IDLE; // Reset
                    break;
                }
                else {
                    rxMsgFrame.data.push_back(byte);
                }
                break;
            case COMS_DECODE_ERROR: // Error State
                comsDecodeState = COMS_DECODE_IDLE;
                break;
            default:
                break;
        };
    }
    
    //@Brief: Deserializes into structs the messages in the queue
    void deserializeMsg() {
        while(topicMsgQueue.empty() == false){
            TopicMsgFrame msgFrame = topicMsgQueue.front(); // Grab the front message
            topicMsgQueue.pop(); // Pop the front message
            std::stringstream ss(msgFrame.data);
            std::string token;
            
            switch(msgFrame.pubID){
                case PUB_STATE:
                    // Deserialize the state message into float vector
                    telemetry_.stateVars.clear();  // Clear previous values
                    for (int i = 0; i < T_NUM_VARS; i++) {
                        std::getline(ss, token, protocol.delim_byte); 
                        try {
                            telemetry_.stateVars.push_back(std::stof(token));
                        } catch (const std::invalid_argument& e) {
                            std::cerr << "Invalid token for float conversion: " << token << std::endl;
                        }
                    }
                    telemetry_.timestamp = std::chrono::high_resolution_clock::now();
                    break;

                case PUB_CMD_RET:
                    cmdret_.msg = msgFrame.data;
                    cmdret_.timestamp = std::chrono::high_resolution_clock::now();
                    if(DEBUG_MODE){ std::cerr << "[DEBUG] Deserializing Command Return" <<  cmdret_.msg << std::endl;}
                    break;

                case PUB_ERROR:
                    logentry_.message = msgFrame.data;
                    logentry_.timestamp = std::chrono::high_resolution_clock::now();
                    if(DEBUG_MODE){ std::cerr << "[DEBUG] Deserializing Log Entry" <<  logentry_.message << std::endl;}
                    break;
                case PUB_INFO:
                    logentry_.message = msgFrame.data;
                    logentry_.timestamp = std::chrono::high_resolution_clock::now();
                    if(DEBUG_MODE){ std::cerr << "[DEBUG] Deserializing Log Entry" <<  logentry_.message << std::endl;}
                    break;
                case (PUB_DEBUG):
                    logentry_.message = msgFrame.data;
                    logentry_.timestamp = std::chrono::high_resolution_clock::now();
                    if(DEBUG_MODE){ std::cerr << "[DEBUG] Deserializing Log Entry" <<  logentry_.message << std::endl;}
                    
                    break;
                default:
                    break;
            }
        }
    }

    //@Brief: Function to format timestamps in human-readable format (with nanosecond precision)
    std::string formatTimestamp(std::chrono::time_point<std::chrono::system_clock> timestamp) {
        auto time_t = std::chrono::system_clock::to_time_t(timestamp);
        std::tm tm = *std::localtime(&time_t);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count() % 1000000000;
        oss << "." << std::setw(9) << std::setfill('0') << ns;
        return oss.str();
    }
    
    // Calculate the time difference between two sample in ms
    double calcSampleTimeDiff(){
        std::chrono::time_point<std::chrono::high_resolution_clock> currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> timeDiff = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime);
        lastTime = currentTime;
        return timeDiff.count() * 1000;
    }

    //@Brief: Configure the serial port  non-canonical mode for reading with select()
    int configSerialPort(const std::string& port, int baud) {
        if (DEBUG_MODE) std::cerr << "[DEBUG] Configuring serial port: " << port << " with baud: " << baud << std::endl;

        int serial_port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // Non-blocking mode
        if (serial_port_fd == -1) {
            std::cerr << "Error: Unable to open serial port " << port << std::endl;
            return -1;
        }
        struct termios options;
        tcgetattr(serial_port_fd, &options);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10; // 1 second timeout
        tcsetattr(serial_port_fd, TCSANOW, &options);

        if (DEBUG_MODE) std::cerr << "[DEBUG] Serial port configured successfully." << std::endl;
        return serial_port_fd;
    }
    

}; 