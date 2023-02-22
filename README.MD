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
    $ docker build -t event-pose --ssh default --build-arg ssh_pub_key="$(cat ~/.ssh/id_rsa.pub)" --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa>)" - < Dockerfile
    ```
:bulb: `<workspace>` is the parent directory in which the repository is cloned

:bulb: The ssh keys are required to access and build this repository from within the docker, as it is currently private. [Create a new ssh key](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) if required.

:warning: Ensure your ssh key is built without a passphrase.

:bulb: Your ssh key may have a different name (e.g. id_ed25519.pub). Change in the command as necessary.

## Usage
- Run the Docker container and, inside it, run the pose detector
    ```shell
    $ xhost +
    $ docker run -it --privileged --network host -v /tmp/.X11-unix/:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb -e DISPLAY=unix$DISPLAY --runtime=nvidia --name event-pose event-pose
    ```

:bulb: use `-v <local_dir>:<container_dir>` to mount further folders inside the container in order to transfer data between your local machine and the container, if needed.

- At the terminal inside the container run the following commands
  ```shell
  $ export ROS_MASTER_URI=http://<localhost-ip>:11311
  $ yarpserver --ros&
  $ atis-bridge-sdk --s 60 --filter 0.01 &
  ```
  :warning: the `&` runs the process in the background enabling a single terminal to run all three processes. Use the command `fg` to bring a background process back to the foreground

- At the terminal inside the container run the following commands. The HPE application should run and a visualisation window should appear. The application should automatically connect to required input and output YARP modules.
  ```shell 
  $ edpr-april
  ```

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

