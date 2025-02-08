#ifndef COMS_H
#define COMS_H
/* PUB_RPC COMS PROTOCOL PC Application Definitions ***************
/*
Pub-RPC Specification Protocol Frame Structure
          |-----------Packet------------|
          ********|---------Frame-------|
********* |-------+---+----+- - - - - - |
 Command  | TOPIC |CMD| ID | DATA       |
          | str   | 1 | 1  | str        | 
********* |-------+---+----+- - - - - - |
 Telemetry| Topic |  DATA  | TIMESTAMP  | 
        | | str   | str    | int64      |     
********* |-------+--------+------+-----|

Note that Command and Telemetry messages are framed differently
*/

#include "common.hpp"

enum RPC {
    GET = 0,
    SET,
    NUM_RPC
};

struct Protocol{ 
    char sof_byte;
    char eof_byte;
    char delim_byte;
    char offset;
}; // Ascii protocol for encoding data

struct TelemetryMsg {
    std::string topic;
    std::string data; // ASCII serialized data
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct Command {
    std::string topic;
    uint8_t cmdID;
    uint8_t paramID;
    std::string data;
};

struct Parameter{
    std::string name; // utility name 
    std::string format; 
    float *reg;
};

// ********* Node Config Manager ********* //
class NodeConfigManager {
private:
    // Mapping for publishers: address -> name
    std::unordered_map<int, std::string> address_to_name;
    // Mapping for parameter configurations keyed by their unique address
    std::unordered_map<int, json> parameter_configs;
    // Mapping for parameter values: address -> Parameter 
    std::unordered_map<int, Parameter> param_map;

public:
    // Constructor that reads the configuration file and extracts the specified section.
    NodeConfigManager(const std::string& filename, const std::string& section) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open config file");
        }
        json config;
        file >> config;

        if (config.contains(section)) {
            // Load publisher information into map
            if (config[section].contains("publishers")) {
                for (const auto& publisher : config[section]["publishers"]) {
                    int address = publisher["address"]; // Get the publisher address
                    std::string name = publisher["name"]; // Get the publisher name
                    address_to_name[address] = name; // Store the publisher name
                }
            }
            // Load parameter configurations into map
            if (config[section].contains("parameters")) {
                for (const auto& param : config[section]["parameters"]) {
                    int address = param["address"]; // Get the parameter address
                    parameter_configs[address] = param; // Store the parameter configuration
                }
            }
        } else {
            throw std::runtime_error("Section '" + section + "' not found in config file");
        }
    }

    // Register a parameter's value pointer by its address.
    void register_parameter(int address, float* reg) {
        try {
            if (parameter_configs.find(address) == parameter_configs.end()) {
                throw std::runtime_error("Parameter at address " + std::to_string(address) + " not found in config");
            }
            struct Parameter *param =  &param_map[address];
            param->name = parameter_configs[address]["name"];
            param->format = parameter_configs[address]["format"];
            param->reg = reg;
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }
    // Get publisher name by address
    std::string get_pub_name(int address) const {
        auto it = address_to_name.find(address);
        return (it != address_to_name.end()) ? it->second : "Unknown Address";
    }
    // Get parameter name by address
    std::string get_param_name(int address) const {
        auto it = parameter_configs.find(address);
        return (it != parameter_configs.end()) ? it->second["name"] : "Unknown Address";
    }
    // Set a parameter's value by its address.
    void set_param(int address, float value) {
        if (param_map.find(address) != param_map.end()) {
            *param_map[address].reg = value; // dereference the pointer and set the value
        } else {
            std::cerr << "Parameter at address " << address << " not registered." << std::endl;
        }
    }
    // Get a parameter's value by its address.
    float get_param(int address) const {
        auto it = param_map.find(address); // iterator to the parameter
        if (it != param_map.end() && it->second.reg != nullptr) {
            return *(it->second.reg); // dereference the pointer and get the value at the register
        }
        std::cerr << "Parameter at address " << address << " not registered." << std::endl;
        return 0.0f;
    }
};

// *********  Utils ********* //
void display_console( const std::string& topic, const std::string& msg, const std::string& timestring) {
    std::cout << "\033[2J\033[1;1H"; // Clear the screen
    fmt::print("[{}] {}: {}\n", timestring, topic, msg);
    std::cout << std::flush; // Keep Latests message at top of screen for UX 
}
inline char proto_encode_id(struct Protocol& proto, uint8_t id) {return id + proto.offset;}
inline uint8_t proto_decode_id(struct Protocol& proto, char id) {return id - proto.offset;}

//@Brief: Packs the cmd string to a cmd frame as per the pub-rpc protocol
std::string proto_pack_asciicmd(const std::string& cmd, struct Protocol& proto) {
    std::string packed_cmd;
    // SOF | DATA | EOF
    packed_cmd.push_back(proto.sof_byte);
    packed_cmd.append(cmd);
    packed_cmd.push_back(proto.eof_byte);
    return packed_cmd;
}

//@Brief: deserializes the command string into a Command struct
//@Note: Command strings are formatted and encoded as per the pub-rpc protocol
//      Example: <BA-0.5 or <AB\n or <BG-1.34:3989:hello\n
//@Note: The command string is parsed in place
bool proto_deserialize_cmd(const std::string& cmd_msg, Protocol &proto, Command& cmd) {
    if (cmd_msg.empty() || cmd_msg[0] != proto.sof_byte) {
        return false; // Invalid command message
    }
    if (cmd_msg.size() < 3) { //  CMDID PARAMID
        return false; // Invalid command message needs atleast 3 bytes SOF CMDID PARAMID
    }
    cmd.cmdID = proto_decode_id(proto, cmd_msg[1]);
    cmd.paramID = proto_decode_id(proto, cmd_msg[2]);
    if (cmd_msg.size() > 3) {
        std::string data = cmd_msg.substr(3); // Extract the data payload
        if (!data.empty() && data.back() == '\n') {
            data.pop_back();   // Remove a trailing newline if present.
        }
        cmd.data = data; // Copy the data payload
    } else {
        cmd.data.clear(); // No data payload is present.
    }
    return true;
}
//@Brief: Forward the telemetry message to the ZMQ publisher socket
//@Note: It is a multipart zmq message <topic><data><timestamp> serialized to ASCII
void zmqcoms_publish_tsmp_msg(zmq::socket_t& pubSocket, const TelemetryMsg& telem) {
    auto duration = telem.timestamp.time_since_epoch();
    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    zmq::message_t topic(telem.topic.size());
    memcpy(topic.data(), telem.topic.data(), telem.topic.size());
    zmq::message_t data(telem.data.size());
    memcpy(data.data(), telem.data.data(), telem.data.size());
    zmq::message_t timestamp(sizeof(uint64_t));
    memcpy(timestamp.data(), &timestamp_ms, sizeof(timestamp_ms));
    // Send the multipart message (topic, data, timestamp)
    pubSocket.send(topic, zmq::send_flags::sndmore);
    pubSocket.send(data, zmq::send_flags::sndmore);
    pubSocket.send(timestamp, zmq::send_flags::none);
}


//@brief: exeuctes the command 
//        and publishes the command return message to the ZMQ socket if needed
//@Note: The command is parsed and executed in place
//@Note: The cmd return message is published to the ZMQ socket in its subtopic  CMD_RET/<node name>
void coms_exec_rpc(const Command& cmd, NodeConfigManager& config, zmq::socket_t& msg_pubsock) {
    float value = 0.0f; // working variable
    
    struct TelemetryMsg cmdret_msg;
    switch(cmd.cmdID) {
        case GET:
            value = config.get_param(cmd.paramID);
            // Publish the telemetry message
            break;
        case SET:
            try{
                value = std::stof(cmd.data);
            } catch (const std::exception& e) {
                std::cerr << " >> Invalid data format: " << cmd.data << std::endl;
                return;
            }
            config.set_param(cmd.paramID, value);
            cmdret_msg.topic = "CMD_RET" + cmd.topic; // Append CMD_RET to the topic
            cmdret_msg.data = std::to_string(value);
            cmdret_msg.timestamp = std::chrono::system_clock::now();
            zmqcoms_publish_tsmp_msg(msg_pubsock, cmdret_msg);
            break;
        default:
            std::cerr << " >> Invalid command ID: " << cmd.cmdID << std::endl;
            return;
    }
}


//@Brief: Receives the Command String
//@Note: Command Strings are formatted and encoded as per the pub-rpc protocol
//@Retrun; topic and data as a tuple
std::tuple<std::string, std::string> zmqcoms_receive_asciicmd(zmq::socket_t& cmdSubSocket) {
    zmq::message_t topic_msg;
    zmq::message_t data_msg;
    (void) cmdSubSocket.recv(topic_msg, zmq::recv_flags::none);
    (void) cmdSubSocket.recv(data_msg, zmq::recv_flags::none); 
    std::string(static_cast<char*>(data_msg.data()), data_msg.size());
    return std::make_tuple(std::string(static_cast<char*>(topic_msg.data()), topic_msg.size()),
                           std::string(static_cast<char*>(data_msg.data()), data_msg.size()));
}

//@Brief: Receives the Telemetry Message from the ZMQ socket
//@Note: Telemetry Messages data are formatted and encoded as per the pub-rpc protocol
//@Note: The message is parsed in place as TOPIC DATA TIMESTAMP as a multipart message
bool zmqcoms_receive_telem(zmq::socket_t& telemSumSocket, TelemetryMsg& telem) {
    zmq::message_t topic_msg;
    zmq::message_t data_msg;
    zmq::message_t timestamp_msg;
    if (!telemSumSocket.recv(topic_msg, zmq::recv_flags::none)) {
        return false;
    }
    if (!telemSumSocket.recv(data_msg, zmq::recv_flags::none)) {
        return false;
    }
    if (!telemSumSocket.recv(timestamp_msg, zmq::recv_flags::none)) {
        return false;
    }
    telem.topic = std::string(static_cast<char*>(topic_msg.data()), topic_msg.size());
    telem.data = std::string(static_cast<char*>(data_msg.data()), data_msg.size());
    // timestamp is sent as a ascii string int value need to convert to int64
    int64_t timestamp_ms = atoi(static_cast<char*>(timestamp_msg.data()));
    try {
        telem.timestamp = std::chrono::time_point<std::chrono::system_clock>(std::chrono::milliseconds(timestamp_ms));
    } catch (const std::exception&) {
        return false;
    }
    telem.timestamp = std::chrono::time_point<std::chrono::system_clock>(std::chrono::milliseconds(timestamp_ms));
    return true;
}

// ********* SERIAL Implementation Pub-RPC
enum SERCOMS_DECODE_TELEM { COMS_TELEM_IDLE = 0, COMS_TELEM_ID, COMS_TELEM_DATA, COMS_TELEM_ERROR};

struct SerComs {
    TelemetryMsg _telem_msg_rx;
    std::queue<TelemetryMsg> telemMsgQ;
    SERCOMS_DECODE_TELEM telemDecodeState;
    std::vector<uint8_t> rxBuffer; // Now owned by SerComs

    // Constructor that initializes the vector with a given size.
    SerComs()
        : rxBuffer(SERCOMS_RX_BUFFER_SIZE) {
        // Additional initialization if needed.
    }
};
//@Brief: Grab the ASCII message frame from the serial port read buffer
//@Description: Parses inplace teh xMacro string mapped to the decoded Publisher ID
//@Note: Multiple messages are pared and pushed to the message queue
//@Note: Partial Messages are completed and verified in the next read
void sercoms_grab_telemetry(int serialFd, struct SerComs& coms, struct Protocol& proto, 
                            NodeConfigManager& config) {
    ssize_t bytesRead = read(serialFd, coms.rxBuffer.data(), coms.rxBuffer.size());
    size_t idx = 0;
    while ((bytesRead - static_cast<ssize_t>(idx)) > 0) {
        uint8_t byte = coms.rxBuffer[idx++];
        switch (coms.telemDecodeState) {
            case COMS_TELEM_IDLE:
                if (byte == proto.sof_byte) {
                    coms._telem_msg_rx = {};  // Reset message
                    coms.telemDecodeState = COMS_TELEM_ID;
                }
                break;
            case COMS_TELEM_ID:
                byte = byte - proto.offset;
                coms._telem_msg_rx.topic = config.get_pub_name(byte);
                coms.telemDecodeState = COMS_TELEM_DATA;
                break;
            case COMS_TELEM_DATA:
                if (byte == proto.sof_byte) {
                    coms.telemDecodeState = COMS_TELEM_ERROR;
                } else if (byte == proto.eof_byte) {
                    coms._telem_msg_rx.timestamp = std::chrono::system_clock::now();
                    coms.telemMsgQ.push(coms._telem_msg_rx);
                    coms.telemDecodeState = COMS_TELEM_IDLE;
                } else {
                    coms._telem_msg_rx.data.push_back(byte);
                }
                break;
            case COMS_TELEM_ERROR:
                coms.telemDecodeState = COMS_TELEM_IDLE;
                break;
        }
    }
}


//@Brief: Parse and Execute RPC commands from the console
int handle_cmdconsole(struct Command& cmd, zmq::socket_t& msg_pubsock, NodeConfigManager& config, Protocol& proto) {
    std::string cmd_input;
    std::getline(std::cin, cmd_input);
    if (cmd_input.empty()) { return 0; }
    cmd_input = proto_pack_asciicmd(cmd_input, proto);
    std::cout << "\033[2J\033[1;1H"; // Clear the screen
    if (cmd_input == "exit") {
        fmt::print("[CONSOLE] Exiting\n");
        return -1;
    }
    TelemetryMsg cmdret_msg;
    fmt::print("<< {}\n", cmd_input);
    if (proto_deserialize_cmd(cmd_input, proto, cmd)) {
        coms_exec_rpc(cmd, config, msg_pubsock);
    } else {
        fmt::print(" >> Invalid Command\n");
    }

    std::cout << std::flush; // Keep Latests message at top of screen for UX
    return 0;
}
//@Breif:Parse and Execute RPC commands from the ZMQ socket
int handle_zmqcmd(struct Command& cmd, zmq::socket_t& cmd_subsock, zmq::socket_t& msg_pubsock,  
                 NodeConfigManager& config, struct Protocol& proto) {
    std::tuple<std::string, std::string> cmd_msg_packet = zmqcoms_receive_asciicmd(cmd_subsock);
    std::string cmd_msg = std::get<1>(cmd_msg_packet);
    display_console(cmd.topic, cmd_msg, formatTimestamp(std::chrono::system_clock::now()));
    if (cmd_msg == "exit") {
        fmt::print("[ZMQ CMD] Exiting\n");
        return -1;
    }
    cmd.topic = std::get<0>(cmd_msg_packet); // Extract the topic to publish back under CMD_RET
    if (proto_deserialize_cmd(cmd_msg, proto, cmd)) {
        coms_exec_rpc(cmd, config, msg_pubsock);
    } else {
        fmt::print("Invalid Command\n");
    }
    return 0;
}


#endif // COMS_H