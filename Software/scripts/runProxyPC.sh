#!bin/bash 

echo "Starting PC <-> dBot proxy server"
./build/zmqproxy 'ipc:///tmp/comsterm_msg' 'tcp://bot.local:5555' & ./build/zmqproxy 'tcp://*:5556' 'ipc:///tmp/vision_cmd' 'ipc:///tmp/comsterm_cmd'

