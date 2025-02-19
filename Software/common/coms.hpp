#ifndef COMS_H
#define COMS_H
/* PUB_RPC COMS PROTOCOL PC Application Definitions ***************

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
#include <shared_mutex>
#include <mutex>
#include <atomic>  // Add this line to include the atomic operations header


enum RPC {GET = 0, SET, NUM_RPC}; // Command IDs

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
struct CommandMsg {
    std::string topic;
    std::string data; // ASCII serialized data
    uint8_t cmdID;
    uint8_t paramID;
};

struct Parameter{
    std::string name; // utility name 
    std::string format; 
    std::atomic<float> *reg; // register to store the value
};


struct ZmqCmdServer{
    zmq::context_t context;
    zmq::socket_t cmd_subsock;
    zmq::socket_t msg_pubsock;
    zmq::socket_t cmd_pubsock;

    ZmqCmdServer(const std::string cmd_substr, const std::string& cmd_socket_address, const std::string& msg_pub_address) 
        : context(1), cmd_subsock(context, zmq::socket_type::sub), 
        msg_pubsock(context, zmq::socket_type::pub), cmd_pubsock(context, zmq::socket_type::pub) {
        
        cmd_subsock.set(zmq::sockopt::linger, 0);
        cmd_subsock.set(zmq::sockopt::subscribe, cmd_substr);
        cmd_subsock.connect(cmd_socket_address); 
        msg_pubsock.set(zmq::sockopt::linger, 0);
        msg_pubsock.connect(msg_pub_address);
        cmd_pubsock.set(zmq::sockopt::linger, 0);
        cmd_pubsock.connect(cmd_socket_address);
    }
};



// ********* Node Config Manager ********* //
class NodeConfigManager {
    private:
        mutable std::shared_mutex param_mutex;                  // Protects param_map
        std::unordered_map<int, std::string> address_to_name;   // Read-only after init
        std::unordered_map<int, json> parameter_configs;        // Read-only after init
        std::unordered_map<int, Parameter> param_map;
    
    public:
        const std::string NODE_NAME;

        //@Brief: Load the node configuration from a JSON file
        NodeConfigManager(const std::string& filename, const std::string& node_name) 
            : NODE_NAME(node_name) {
            std::ifstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("Failed to open config file");
            }
            json config;
            file >> config; // Load the JSON file
            if (config.contains(NODE_NAME)) {
                if (config[NODE_NAME].contains("publishers")) { // Load publisher information into map
                    for (const auto& publisher : config[NODE_NAME]["publishers"]) {
                        int address = publisher["address"];    
                        std::string name = publisher["name"];
                        address_to_name[address] = name;
                    }
                }
                if (config[NODE_NAME].contains("parameters")) { // Load parameter configurations into map
                    for (const auto& param : config[NODE_NAME]["parameters"]) {
                        int address = param["address"]; 
                        parameter_configs[address] = param; 
                    }
                }
            } else {
                throw std::runtime_error("Section '" + NODE_NAME + "' not found in config file");
            }
        }
        //@Brief: Register the parameter with the config manager
        //@Note: The parameter must be present in the config file
        void register_parameter(int address, std::atomic<float>* reg) {
            std::unique_lock<std::shared_mutex> lock(param_mutex); // Exclusive access to param_map
            
            if (parameter_configs.find(address) == parameter_configs.end()) {
                throw std::runtime_error("Parameter at address " +  std::to_string(address) + " not found in config");
            }
    
            Parameter param;
            param.name = parameter_configs[address]["name"];
            param.format = parameter_configs[address]["format"];
            param.reg = reg;
            
            param_map[address] = param;  // Actually store the parameter
        }
    
       //@Brief: Get the name of the parameter at the given address
       //@Returns: The name of the parameter or "Unknown Address" if not found
       //@Note: This function is thread-safe
        std::string get_param_name(int address) const {
            auto it = parameter_configs.find(address);
            return (it != parameter_configs.end()) ? it->second["name"] : "Unknown Address";
        }
        //@Brief: Get the name of the publisher at the given address
        //@Returns: The name of the publisher or "Unknown Address" if not found
        //@Note: This function is thread-safe
        std::string get_pub_name(int address) const {
            auto it = address_to_name.find(address);
            return (it != address_to_name.end()) ? it->second : "Unknown Address";
        }
        //@Brief: Set the parameter at the given address
        //@Note: This function is thread-safe
        void set_param(int address, float value) {
            std::shared_lock<std::shared_mutex> lock(param_mutex);
            if (auto it = param_map.find(address); it != param_map.end()) {
                it->second.reg->store(value, std::memory_order_release);
                fmt::print("[{}] Set {} to {}\n", NODE_NAME, it->second.name, value);
            } else {
                std::cerr << "Parameter at address " << address 
                         << " not registered.\n";
            }
        }
        //@Brief: Get the parameter at the given address
        //@Returns: The value of the parameter or 0.0f if not found
        //@Note: This function is thread-safe
        float get_param(int address) const {
            std::shared_lock<std::shared_mutex> lock(param_mutex);
            if (auto it = param_map.find(address); 
                it != param_map.end() && it->second.reg != nullptr) 
            {
                return it->second.reg->load(std::memory_order_acquire);
            }
            std::cerr << "Parameter at address " << address << " not registered.\n";
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


void proto_pack_asciicmd(std::string& cmd, const struct Protocol& proto) {
    // SOF | DATA | EOF
    cmd.insert(0, 1, proto.sof_byte);
    cmd.push_back(proto.eof_byte);
}

//@Brief: deserializes the command string into a Command struct
//@Note: Command strings are formatted and encoded as per the pub-rpc protocol
//      Example: <BA-0.5 or <AB\n or <BG-1.34:3989:hello\n
//@Note: The command string is parsed in place
//@Returns: true if the command message is parsed successfully
bool proto_deserialize_cmd(const std::string& cmd_msg, Protocol &proto, CommandMsg& cmd) {
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
    return true; // Command message parsed successfully
}
//@Brief: Forward the telemetry message to the ZMQ publisher socket
//@Note: It is a multipart zmq message <topic><data><timestamp> serialized to ASCII
void zmqcoms_publish_tsmp_msg(zmq::socket_t& pubSocket, const TelemetryMsg& telem, std::string nodeName) {
    auto duration = telem.timestamp.time_since_epoch();
    // Prepend node name 
    std::string topicstr = nodeName + "/" + telem.topic;
    zmq::message_t topic(topicstr.size());
    memcpy(topic.data(), topicstr.data(), topicstr.size());
    zmq::message_t data(telem.data.size());
    memcpy(data.data(), telem.data.data(), telem.data.size());
    // format timestamp as ascii string
    auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    std::string timestamp = std::to_string(timestamp_ms);
    zmq::message_t timestamp_msg(timestamp.size());
    memcpy(timestamp_msg.data(), timestamp.data(), timestamp.size());
    // Send the multipart message (topic, data, timestamp)
    pubSocket.send(topic, zmq::send_flags::sndmore);
    pubSocket.send(data, zmq::send_flags::sndmore);
    pubSocket.send(timestamp_msg, zmq::send_flags::none);
}


//@brief: executes the command 
//        and publishes the command return message to the ZMQ socket if needed
//@Note: The command is parsed and executed in place
//@Note: The cmd return message is published to the ZMQ socket in its subtopic  CMD_RET/<node name>
void coms_exec_rpc(const CommandMsg& cmd, NodeConfigManager& config, zmq::socket_t& msg_pubsock) {
    float value = 0.0f; // working variable
    
    struct TelemetryMsg cmdret_msg;
    switch(cmd.cmdID) {
        case GET:
            value = config.get_param(cmd.paramID);
            // build the cmd return message
            cmdret_msg.topic = "CMD_RET" + cmd.topic; // Append CMD_RET to the topic
            cmdret_msg.data = std::to_string(value);
            cmdret_msg.timestamp = std::chrono::system_clock::now();
            zmqcoms_publish_tsmp_msg(msg_pubsock, cmdret_msg, config.NODE_NAME);
            break;
        case SET:
            try{
                value = std::stof(cmd.data);
            } catch (const std::exception& e) {
                std::cerr << " >> Invalid data format: " << cmd.data << std::endl;
                return;
            }
            config.set_param(cmd.paramID, value);
            
  
            break;
        default:
            std::cerr << " >> Invalid command ID: " << cmd.cmdID << std::endl;
            return;
    }
}

//@Brief: Receives the Command String
//@Note: Command Strings are formatted and encoded as per the pub-rpc protocol
void zmqcoms_receive_asciicmd(zmq::socket_t& cmdSubSocket, struct CommandMsg& cmdFrame) {
    zmq::message_t topic_msg;
    zmq::message_t cmd_ret_data;
    (void) cmdSubSocket.recv(topic_msg, zmq::recv_flags::none);
    (void) cmdSubSocket.recv(cmd_ret_data, zmq::recv_flags::none); 
    // pack the data into the cmd frame
    cmdFrame.topic = std::string(static_cast<char*>(topic_msg.data()), topic_msg.size());
    cmdFrame.data = std::string(static_cast<char*>(cmd_ret_data.data()), cmd_ret_data.size());
}

//@Brief: Receives the Telemetry Message from the ZMQ socket
//@Note: Telemetry Messages data are formatted and encoded as per the pub-rpc protocol
//@Note: The message is parsed in place as TOPIC DATA TIMESTAMP as a multipart message
bool zmqcoms_receive_telem(zmq::socket_t& telemSumSocket, TelemetryMsg& telem) {
    zmq::message_t topic_msg;
    zmq::message_t data_msg;
    zmq::message_t timestamp_msg;
    if (!telemSumSocket.recv(topic_msg, zmq::recv_flags::none)) {return false;}
    if (!telemSumSocket.recv(data_msg, zmq::recv_flags::none)) {return false;}
    if (!telemSumSocket.recv(timestamp_msg, zmq::recv_flags::none)) {return false;}

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

//@Brief:Parse and Execute RPC commands from the ZMQ socket
//@Returns: -1 if exit command is received
int handle_zmqcmd(struct CommandMsg& cmdmsg, zmq::socket_t& cmd_subsock, zmq::socket_t& msg_pubsock,  zmq::socket_t& cmd_pubsock,
                 NodeConfigManager& config, struct Protocol& proto) {
    zmqcoms_receive_asciicmd(cmd_subsock, cmdmsg);
    if (cmdmsg.data == "exit") {
        fmt::print("[ZMQ CMD] Exiting\n");
        return -1;
    }
    if (!proto_deserialize_cmd(cmdmsg.data, proto, cmdmsg)) { return -1;}
    coms_exec_rpc(cmdmsg, config, msg_pubsock);
    return 0;
}

//@Brief: Parse the console input and pack it into a command frame
//@Returns: -1 if exit command is received
int preprocess_console_input(std::string& cmd_input, struct Protocol& proto) {
    if (cmd_input.empty()) { return 0; }
    if (cmd_input == "exit") {return -1;}
    proto_pack_asciicmd(cmd_input, proto);
    return 0;
}


#endif // COMS_H