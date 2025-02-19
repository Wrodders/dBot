#!/bin/bash
# -------------------------------------------------- 
#  @file    runProxyPC.sh
#  @brief   Starts the proxy server on the PC

echo "Starting PC <-> dBot proxy server"

./build/zmqproxy 'tcp://*:5556' 'ipc:///tmp/botcmds' & ./build/zmqproxy 'ipc:///tmp/botmsgs' 'tcp://bot.local:5555'
