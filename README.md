# EDPR-APRIL
EDPR repository for the European project [APRIL](http://aprilproject.eu/).

## Installation
The software was tested on Ubuntu 20.04.2 LTS.

- Install the latest [Nvidia driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver)
- Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu)
- Install [Nvidia Docker Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- Build the Docker image
```shell
$ docker build -t <container_name> - < Dockerfile
```

## Usage
- Open a terminal, run the Docker container and the yarp server
    ```shell
    $ xhost +
    $ docker run -it -v /path/to/code/dir:/code -v /tmp/.X11-unix/:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --runtime=nvidia <image_id>
    # inside the container
    $ yarpserver
    ```

- Open a new terminal and run the frames producer in the running container
    ```shell
    $ docker exec -it <container_id/container_name> bash  # executes a bash terminal in the running container
    # inside the container
    $ yarpdev --device fakeFrameGrabber --period 0.5 --width 640 --height 480 --name /frameGrabber
    ```

- Open a new terminal, compile and run the pose detector in the running container
    ```shell
    $ docker exec -it <container_id/container_name> bash  # executes a bash terminal in the running container
    # inside the container
    $ cd /code
    $ mkdir build && cd build
    $ cmake .. && make
    $ ./test.bin --yarp-image-producer /frameGrabber --model-folder /openpose/models
    ```
