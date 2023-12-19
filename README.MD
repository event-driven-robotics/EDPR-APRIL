# EDPR-APRIL

Human Pose Estimation for Ergonomics and Fatigue Detection


*Primary components:*
- Can use detectors: moveEnet
- Pixelwise velocity estimation
- Fusion of position and velocity estimation for latency compensated final position output

The application has been designed to run using docker for simple set-up of the environment.

## Installation
The software was tested on Ubuntu 20.04.2 LTS with an Nvidia GPU.

- Install the latest [Nvidia driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver)
- Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu)
- Install [Nvidia Docker Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- Download the repository and build the Docker image
    ```shell
    $ cd <workspace>
    $ git clone git@github.com:event-driven-robotics/EDPR-APRIL.git
    $ cd EDPR-APRIL
    $ docker build -t april-event-pose - < Dockerfile
    ```
:bulb: `<workspace>` is the parent directory in which the repository is cloned

## Usage
- Run the Docker container and, inside it, run the pose detector
    ```shell
    $ xhost +
    $ docker run -it --privileged --network host -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb -e DISPLAY=unix$DISPLAY --gpus all --name april-event-pose april-event-pose
    ```

:bulb: use `-v <local_dir>:<container_dir>` to mount further folders inside the container in order to transfer data between your local machine and the container, if needed.

- At the terminal inside the container run the following command to execute a script that will set up a yarpserver connected to ROS, run the atis-bridge to get the events from the camera and finally run the HPE application with a visualisation window. The application should automatically connect to required input and output YARP modules. This script also sets the value to the env variable `ROS_MASTER_URI`. It is either set to the default valaue (i.e. `http://127.0.0.1:11311`) using the `-d` flag, or manually set to a differnet value using the flag `-m`.
  
```shell 
$ ./run_april [-d (default)] or [-m value (manual)]
```
To stop the application itself, the camera and the yarpserver, the following command should be run in a temrinal inside the docker:

```shell 
$ ./kill_april
```

### human pose estimation: edpr-april
- The application should run with default parameters, using moveEnet detector at 10 Hz, without the latency compensation.

- While the visualisation window is selected:
  - Press `d` to visualise the detections (on and off)
  - Press `v` to visualise the joint velocities (on and off)
  - Press `e` to visualise alternate representations (toggle between eros and events)
  - Press `ESC` to close

- The following command line options can be used `edpr-april --OPTION VALUE`
  - `--use_lc` to use latency compensation in the fusion
  - `--detF <INT>` to modify the moveEnet detection rate [10]
  - `--w <INT> --h <INT>` to set the dataset sensor resolution [640 480]
  - `--roi <INT>` to set the joint region-of-interest [20]
  - `--pu <FLOAT> --muD <FLOAT>` to set the Kalman filter process and measurement uncertainty [0.001, 0.0004]
  - `--sc <FLOAT>` to change a tuning parameter associated with velocity [10]
  - `--filepath <STRING>` to save a `.csv` of joint positions at a provided file location
  - `--velpath  <STRING>` to save a `.csv` of joint velocities at a provided file location
  - `--v <STRING>` to save a video at a provided file location

### emergency safety: visual-fault-button

The emergency fault button runs without arguments and has an interactive calibration procedure to set correct parameters for the environment and scenario. Follow onscreen prompts to start, calibration and run the neuromorphic visual fault button.

