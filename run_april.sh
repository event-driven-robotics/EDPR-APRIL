#!/bin/bash

#export ROS_MASTER_URI=http://127.0.0.1:11311

echo "Run YARP server connected to ROS"
yarpserver --ros &
sleep 2
echo "Run ATIS-bridge"
atis-bridge-sdk --s 40 --filter 0.01 &
sleep 5
echo "Run EDPR-APRIL application"
edpr-april --thF 30  --detF 10 --movenet --gpu --ve pxt --ros &
sleep 4
echo "Run Visual Fault Button application"
visual-fault-button --usecase $USECASE &
