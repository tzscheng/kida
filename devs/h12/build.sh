#!/bin/bash

gcc -W -Wall -o check check.c -lpthread


#gcc -W -Wall -o basic basic.c -lpthread -lm


#gcc -W -Wall -shared -o io-can.so -fPIC io-can.c  -include sockcan.h


gcc -W -Wall -shared -o eio.so eio.c -fPIC -I/usr/local/include/dynamixel_sdk -ldxl_x64_c


#gcc -W -Wall -o udp2zmq udp2zmq.c -lzmq -lm -I/home/ubuntu/fg/tact
