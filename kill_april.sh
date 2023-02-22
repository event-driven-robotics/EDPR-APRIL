#!/bin/bash
echo "Killing all APRIL tasks"
echo "======================="
killall python3
echo "Killed MoveEnet"
killall edpr-april
echo "Killed EDPR APRIL app"
killall atis-bridge-sdk
echo "ATIS-bridge killed"
killall yarpserver
echo "YARP server killed"
