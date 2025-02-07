#!bin/bash 

echo "Starting PC <-> dBot proxy server"
./build/zmqproxy 'tcp://*:5555' 'ipc:///tmp/botmsgs'  || error_exit "Fail Launch ZMQ Proxy" & ./build/zmqproxy 'ipc:///tmp/botcmds' 'tcp://Rodrigos-MacBook-Air.local:5556'  || error_exit "Fail launch ZMQ Proxy"

