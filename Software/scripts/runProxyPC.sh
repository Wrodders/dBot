#!bin/bash 

echo "Starting PC <-> dBot proxy server"

./build/zmqproxy 'tcp://*:5556' 'ipc:///var/tmp/comsterm_cmd' & ./build/zmqproxy 'ipc:///var/tmp/comsterm_msg' 'tcp://bot.local:5555'
