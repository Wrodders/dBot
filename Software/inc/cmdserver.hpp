#ifndef CMD_SERVER_HPP
#define CMD_SERVER_HPP

#include "../common/coms.hpp"
#include "vision.hpp"


namespace cmd {
//@brief: Command Server
//@description: Listens for commands on the VISION topic over TCP and IPC; publishes command responses to the VISION topic over TCP
void command_server(ParameterMap& param_map) {
    syslog(LOG_INFO, "Starting Command Server");
    Protocol _proto = { '<', '\n', ':', 'A' };
    CommandMsg recv_cmd_msg;
    zmq::context_t context(1);

    zmq::socket_t cmd_subsock(context, zmq::socket_type::sub);
    cmd_subsock.set(zmq::sockopt::linger, 0);
    cmd_subsock.set(zmq::sockopt::subscribe, "VISION");
    cmd_subsock.connect("ipc:///tmp/botcmds");
    syslog(LOG_INFO, "Subscribed Cmd Server to ipc:///tmp/botcmds");

    zmq::socket_t msg_pubsock(context, zmq::socket_type::pub);
    msg_pubsock.set(zmq::sockopt::linger, 0);
    msg_pubsock.bind("ipc:///tmp/botmsgs");
    syslog(LOG_INFO, "Bound Msg Server to ipc:///tmp/botmsgs");

    while (!viz::_exit_trig) {
        coms_receive_asciicmd(cmd_subsock, recv_cmd_msg);
        coms_handle_cmd(recv_cmd_msg, msg_pubsock, param_map, _proto, "VISION");
    }
    syslog(LOG_INFO, "Exiting Command Server");
    context.close();
    viz::_exit_trig.store(true);
}

// -------------- Parameter Validation -------------- //
static inline bool val_hrz_height(float val) { return (val > 0 && val < viz::HEIGHT); }
static inline bool val_trfm_pad(float val)   { return (val > 0 && val < viz::WIDTH); }
static inline bool val_prog_mode(float val)  { return (val >= 0 && val < viz::NUM_MODES); }
static inline bool val_max_vel(float val)    { return (val >= 0 && val < 1); }
static inline bool val_nav_en(float val)     { return (static_cast<int>(val) % 2 == 0 || static_cast<int>(val) % 2 == 1); } // test if 0 or 1
static inline bool val_kp(float val)        { return (val >= 0 && val < 100); } // test if 0 or 1

} // namespace cmd

#endif // CMD_SERVER_HPP