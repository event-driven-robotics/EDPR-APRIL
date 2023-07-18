#!/bin/bash
echo "Killing all APRIL tasks"
echo "======================="
killall visual-fault-button
echo "Killed visual-fault-button app"
killall edpr-april
echo "Killed EDPR APRIL app"
killall python3
echo "Killed MoveEnet"
killall atis-bridge-sdk
echo "ATIS-bridge killed"
killall yarpserver
echo "YARP server killed"
