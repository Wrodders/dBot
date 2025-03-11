#ifndef COMS_H
#define COMS_H
/* PUB_RPC COMS PROTOCOL PC Application Definitions ***************

Pub-RPC Specification Protocol Frame Structure
          |-----------Packet---------------------|
          | Routing Header |---------Frame-------|
********* |----------------+---+----+- - - - - - |
 Command  | NodeName       |CMD| ID | DATA       |
          | str            | 1 | 1  | str        | 
********* |----------------+---+----+- - - - - - |
 Message  | Topic/Nodename |  DATA  | TIMESTAMP  | 
        | | str            | str    | int64      |     
********* |----------------+--------+------------|

Note that Command and Telemetry messages are framed differently
*/

#include "common.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <fstream>
#include <atomic> 
#include <exception>
#include <nlohmann/json.hpp>



using json = nlohmann::json;

struct TopicInfo{
    uint8_t ID;
    std::string name;
    std::vector<std::string> args_names;
    std::string format;
    std::string description;
    int nArgs; // utility
}; // Topic Info

//@Brief: Utility class to manage the topics of a particular node
//@Note: The topics are loaded from a JSON file
class PubMap {
    private:
        std::vector<TopicInfo> topics_by_id;
        std::unordered_map<std::string, uint8_t> name_to_id;    
        bool load_topics(const std::string& filename, const std::string& node_name) {
            try {
                std::ifstream file(filename);
                if (!file.is_open()) {
                    throw std::runtime_error("Failed to open config file");
                }
                json config;
                file >> config; // Load the JSON file
                if (config.contains(node_name)) {
                    auto& publishers = config[node_name]["publishers"];
                    topics_by_id.resize(publishers.size());
    
                    for (const auto& topic : publishers) {
                        TopicInfo t;
                        t.ID = topic["id"];
                        t.name = topic["name"];
                        t.format = topic["format"];
                        t.description = topic["description"];
                    
                        for (const auto& arg : topic["args"]) {
                            t.args_names.push_back(arg);
                        }
                    
                        t.nArgs = t.args_names.size();
                    
                        if (t.ID >= topics_by_id.size())
                            topics_by_id.resize(t.ID + 1);
                    
                        topics_by_id[t.ID] = t;
                        name_to_id[t.name] = t.ID;
                    }
                }
                return true;
            } catch (const nlohmann::json::exception& e) {
                std::cerr << "JSON error: " << e.what() << std::endl;
                return false;
            }
        }
    
    public:
        PubMap(const std::string& filename, const std::string& node_name) {
            if (!load_topics(filename, node_name)){
                throw std::runtime_error("Failed to load topics");
            }

        }
    
        const TopicInfo* get_topic(uint8_t id) const {
            return (id < topics_by_id.size()) ? &topics_by_id[id] : nullptr;
        }
    
        const TopicInfo* get_topic(const std::string& name) const {
            auto it = name_to_id.find(name);
            return (it != name_to_id.end()) ? get_topic(it->second) : nullptr;
        }

        const std::string get_topic_name(uint8_t id) const {
            return (id < topics_by_id.size()) ? topics_by_id[id].name : "";
        }   


    };
    

struct ParameterInfo{
    uint8_t ID;     // Allows for 256 parameters
    std::string name; 
    std::string format;
    std::string access; // Read or Write
    std::string description; // tooltip
   // function pointer to validator function 
    bool (*validator)(float);
};

//@Brief: Utility class to manage the parameters of a node
//@Note: The parameters info are loaded from a JSON file
class ParameterMap {
    private:
        std::vector<ParameterInfo> parameters_by_id;
        std::vector<std::atomic<float>*> param_registers;
        std::unordered_map<std::string, uint8_t> name_to_id;

        bool load_parameters(const std::string& filename, const std::string& node_name) {
            try {
                std::ifstream file(filename);
                if (!file.is_open()) {
                    throw std::runtime_error("Failed to open config file");
                }
                json config;
                file >> config; // Load the JSON file
                if (config.contains(node_name)) {
                    auto& parameters = config[node_name]["parameters"];
                    parameters_by_id.resize(parameters.size());
    
                    for (const auto& topic : parameters) {
                        ParameterInfo p;
                        p.ID = topic["id"];
                        p.name = topic["name"];
                        p.format = topic["format"];
                        p.access = topic["access"];
                        p.description = topic["description"];

                        if (p.ID >= parameters_by_id.size())
                            parameters_by_id.resize(p.ID + 1);

                        parameters_by_id[p.ID] = p;
                        name_to_id[p.name] = p.ID;
                    }            
                }
                return true;
            } catch (const nlohmann::json::exception& e) {
                std::cerr << "JSON error: " << e.what() << std::endl;
                return false;
            }
        }

    
    public:
        ParameterMap(const std::string& filename, const std::string& node_name) {
            if (!load_parameters(filename, node_name)) {
                throw std::runtime_error("Failed to load parameters");
            }
        }

        bool register_parameter(uint8_t id, std::atomic<float>& value, bool (*validator)(float)) {
            if (id >= parameters_by_id.size()) return false;
            param_registers.resize(parameters_by_id.size());
            param_registers[id] = &value;
            parameters_by_id[id].validator = validator;
            return true;
        }
    
        const ParameterInfo* get_parameter(uint8_t id) const {
            return (id < parameters_by_id.size()) ? &parameters_by_id[id] : nullptr;
        }
    
        const ParameterInfo* get_parameter(const std::string& name) const {
            auto it = name_to_id.find(name);
            return (it != name_to_id.end()) ? get_parameter(it->second) : nullptr;
        }
    
        bool set_value(uint8_t id, float value) {
            if (id >= param_registers.size()) {return false;}
            if(!param_registers[id]) {return false;}
            if(!parameters_by_id[id].validator(value)) {return false;}
            param_registers[id]->store(value, std::memory_order_relaxed);
            return true;
        }
    
        bool get_value(uint8_t id, float& value) const {
            if (id >= param_registers.size()) return false;
            if(!param_registers[id]) {return false;}
            value = param_registers[id]->load(std::memory_order_relaxed);
            return true;
        }
    
        bool set_value(const std::string& name, float value) {
            auto it = name_to_id.find(name);
            if(it == name_to_id.end()) {return false;}
            if(!param_registers[it->second]) {return false;}
            if(!parameters_by_id[it->second].validator(value)) {return false;}
            param_registers[it->second]->store(value, std::memory_order_relaxed);
            return true;
        }
    
        bool get_value(const std::string& name, float& value) const {
            auto it = name_to_id.find(name);
            if(it == name_to_id.end()) {return false;}
            if(!param_registers[it->second]) {return false;}
            value = param_registers[it->second]->load(std::memory_order_relaxed);
            return true;
        }
    };

// ---------- PUB_RPC Utilties ----------------- //
enum RPC {GET = 0, SET, NUM_RPC}; // Command IDs
struct Protocol{ 
    char sof_byte;
    char eof_byte;
    char delim_byte; 
    char offset;
}; // Ascii protocol for encoding data

struct Message {
    std::string topic;
    std::string data; // ASCII serialized data
    std::string time_str; // ns timestamp
    
};
struct CommandMsg {
    std::string topic;
    std::string data; // ASCII serialized data
    uint8_t cmdID;
    uint8_t paramID;
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

//@Brief: Forwards the telemetry message to the ZMQ publisher socket
//@Note: It is a multipart zmq message <topic><data><timestamp> serialized to ASCII
//@Note: 
void coms_publish_tsmp_msg(zmq::socket_t& pubSocket,const Message& msg) {
    zmq::message_t topic(msg.topic.size());
    memcpy(topic.data(), msg.topic.data(), msg.topic.size());
    zmq::message_t data(msg.data.size());
    memcpy(data.data(), msg.data.data(), msg.data.size());
    zmq::message_t timestamp_msg(msg.time_str.size());
    memcpy(timestamp_msg.data(), msg.time_str.data(), msg.time_str.size());
    // Send the multipart message (topic, data, timestamp)
    pubSocket.send(topic, zmq::send_flags::sndmore);
    pubSocket.send(data, zmq::send_flags::sndmore);
    pubSocket.send(timestamp_msg, zmq::send_flags::none);
}


//@brief: executes the command 
//        and publishes the command return message to the ZMQ socket if needed
//@Note: The command is parsed and executed in place
//@Note: The cmd return message is published to the ZMQ socket in its subtopic  CMD_RET/<node name>
void coms_exec_rpc(const CommandMsg& cmd, ParameterMap& param_map, zmq::socket_t& msg_pubsock, const std::string& node_name) {
    float value = 0.0f; // working variable
    
    struct Message cmdret_msg;
    switch(cmd.cmdID) {
        case GET:
            if(!param_map.get_value(cmd.paramID, value)){
                syslog(LOG_WARNING, "Invalid parameter ID: %d", cmd.paramID);
                return;
            }
            // build the cmd return message
            cmdret_msg.topic = "CMD_RET/" + node_name;
            cmdret_msg.data = std::to_string(value);
            cmdret_msg.time_str = timestamp();
            coms_publish_tsmp_msg(msg_pubsock, cmdret_msg);
            break;
        case SET:
            try{
                value = std::stof(cmd.data);
            } catch (const std::exception& e) {
               syslog(LOG_WARNING, "Invalid data value: %s", cmd.data.c_str());
                return;
            }
            if(!param_map.set_value(cmd.paramID, value)){
                syslog(LOG_WARNING, "Invalid parameter ID: %d", cmd.paramID);
                return;
            }
            break;
        default:
            syslog(LOG_WARNING, "Invalid Command ID: %d", cmd.cmdID);
            return;
    }
}

//@Brief: Receives the Command String
//@Note: Command Strings are formatted and encoded as per the pub-rpc protocol
void coms_receive_asciicmd(zmq::socket_t& cmdSubSocket, struct CommandMsg& cmdFrame) {
    zmq::message_t topic_msg;
    zmq::message_t cmd_ret_data;
    zmq::recv_result_t result = cmdSubSocket.recv(topic_msg, zmq::recv_flags::none);
    if (!result) {
        syslog(LOG_WARNING, "Failed to receive command topic");
        return;
    }
    result = cmdSubSocket.recv(cmd_ret_data, zmq::recv_flags::none);
    if (!result) {
        syslog(LOG_WARNING, "Failed to receive command data");
        return;
    }
    // pack the data into the cmd frame
    cmdFrame.topic = std::string(static_cast<char*>(topic_msg.data()), topic_msg.size());
    cmdFrame.data = std::string(static_cast<char*>(cmd_ret_data.data()), cmd_ret_data.size());
}


//@Brief:Parse and Execute RPC commands from the ZMQ socket
//@Returns: -1 if exit command is received
void coms_handle_cmd(struct CommandMsg& cmdmsg, zmq::socket_t& msg_pubsock,
                 ParameterMap& param_map, struct Protocol& proto, const std::string& node_name) {

    if (!proto_deserialize_cmd(cmdmsg.data, proto, cmdmsg)) { 
        syslog(LOG_WARNING, "Invalid Command Message: %s", cmdmsg.data.c_str());
        return;
    }
    coms_exec_rpc(cmdmsg, param_map, msg_pubsock, node_name);
    
}
#endif // COMS_H
