#!/bin/bash


g++ -o /build/twsbcoms tools/twsbComs.cpp $(pkg-config --cflags --libs opencv4 libzmq fmt) -std=c++17 -pthread
g++ -o /build/zmqPubTest Test/zmqPubTest.cpp $(pkg-config --cflags --libs opencv4 libzmq fmt) -std=c++17 -pthread
g++ -o /build/zmqSubTest Test/zmqSubTest.cpp $(pkg-config --cflags --libs opencv4 libzmq fmt) -std=c++17 -pthread
g++ -o /build/zmqproxy tools/zmqProxy.cpp $(pkg-config --cflags --libs opencv4 libzmq fmt) -std=c++17 -pthread

