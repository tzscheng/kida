#check
#cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

#update
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
