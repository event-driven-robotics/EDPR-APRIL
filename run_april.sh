#!/bin/bash
export ROS_MASTER_URI=http://127.0.0.1:11311
echo "Run YARP server connected to ROS"
yarpserver --ros &
echo "Run ATIS-bridge"
atis-bridge-sdk --s 60 --filter 0.01 &
echo "Run EDPR-APRIL application"
edpr-april
