#!/bin/bash

#export ROS_MASTER_URI=http://127.0.0.1:11311

echo "Run YARP server connected to ROS"
yarp namespace /april
# yarp conf ${ROS_MASTER_URI:7:-6} 10000
yarp conf ${YARPSERVER_IP} 10000
yarpserver --ros &
sleep 2
echo "Run ATIS-bridge"
atis-bridge-sdk --gen3 --s ${P_CAMS} --filter ${P_CAMF} &
sleep 5
echo "Run EDPR-APRIL application"
edpr-april --f_vis ${P_VISF}  --f_det ${P_DETF} --confidence ${P_CONF} &
sleep 4
echo "Run Visual Fault Button application"

if [ -z ${USECASE+x} ]; then 
    echo "USECASE is unset";
    USECASE="latest"
fi
fault_calib_path="/usr/local/src/EDPR-APRIL/fault_button/calibrations/${USECASE}_calibration.txt"
visual-fault-button --calib_path $fault_calib_path &
