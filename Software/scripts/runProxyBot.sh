#!bin/bash 

echo "Starting PC <-> dBot proxy server"
./build/zmqproxy 'tcp://*:5555' 'ipc:///tmp/botMSGS'

