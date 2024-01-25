#!/bin/bash

gcc -o serialReader serialReader.c -lserialport -L/opt/homebrew/Cellar/libserialport/0.1.1/lib -I/opt/homebrew/Cellar/libserialport/0.1.1/include