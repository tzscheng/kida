#!/bin/bash

#motor/encoder test program
#gcc -W -Wall -o check check.c -include ../can/can-113.h -I/usr/include/ixxat -lpthread -leci113DriverLinux-usb-1.0
gcc -W -Wall -o check check.c -lpthread

#main controller
#gcc -W -Wall -o basic basic.c -include ../can/can-113.h -I/usr/include/ixxat -lpthread -leci113DriverLinux-usb-1.0 -lm
#gcc -W -Wall -o basic basic.c -include sockcan.h -lpthread -lm

#io
gcc -W -Wall -shared -o eio.so eio.c -fPIC
