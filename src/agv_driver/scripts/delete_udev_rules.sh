#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  agv_driver"
echo "sudo rm   /etc/udev/rules.d/agv.rules hokuyo.rules"
sudo rm /etc/udev/rules.d/agv.rules
sudo rm /etc/udev/rules.d/50-hokuyo.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
