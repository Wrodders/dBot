#!bin/bash 

# Start the proxy server
./build/zmqproxy ipc:///tmp/comsterm_cmd ipc:///tmp/vision_cmd tcp://*:5556 --debug
