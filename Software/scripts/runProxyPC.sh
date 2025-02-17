#!/bin/bash
# -------------------------------------------------- 
#  @file    runProxyPC.sh
#  @brief   Starts the proxy server on the PC
#  @date    2025-01-05
#  @version 1.0

echo "Starting PC <-> dBot proxy server"

./build/zmqproxy 'tcp://*:5556' 'ipc:///tmp/botcmds' & ./build/zmqproxy 'ipc:///tmp/botmsgs' 'tcp://dbot.local:5555'
