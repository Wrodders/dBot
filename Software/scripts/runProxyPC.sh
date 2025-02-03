#!bin/bash 

echo "Starting PC <-> dBot proxy server"

./build/zmqproxy 'tcp://*:5556' 'ipc:///tmp/comsterm_cmd' \
& ./build/zmqproxy 'ipc:///tmp/comsterm_msg' 'tcp://bot.local:5555'
