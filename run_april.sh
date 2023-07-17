#!/bin/bash

while getopts 'dm:' OPTION; do
    case "$OPTION" in
    d)
        echo "ROS_MASTER_URI set to default value"
        export ROS_MASTER_URI=http://127.0.0.1:11311
        echo "ROS_MASTER_URI="$ROS_MASTER_URI
        ;;
    m)
        echo "ROS_MASTER_URI set manually"
        export ROS_MASTER_URI="$OPTARG"
        echo "ROS_MASTER_URI="$ROS_MASTER_URI
        ;;
    ?)
        echo "script usage: /run_april.sh [-d (default)] or [-m value (manual)]" >&2
        exit 1
        ;;
    esac
done

if [ "$#" -eq  "0" ]
   then
    echo "ROS_MASTER_URI not set"
    exit 1
fi

echo "Run YARP server connected to ROS"
yarpserver --ros &
echo "Run ATIS-bridge"
atis-bridge-sdk --s 60 --filter 0.01 &
echo "Run EDPR-APRIL application"
edpr-april --thF 30  --detF 10 --movenet --gpu --ve pxt --ros
