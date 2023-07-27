#!/bin/bash
echo "Killing all APRIL tasks"
echo "======================="
killall visual-fault-button
echo "Killed visual-fault-button app"
sleep 3
killall edpr-april
echo "Killed EDPR APRIL app"
sleep 3
killall python3
echo "Killed all Python3 tasks (should be moveEnet)"
killall atis-bridge-sdk
echo "ATIS-bridge killed"
sleep 3
killall yarpserver
echo "YARP server killed"
