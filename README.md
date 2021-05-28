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
- Place video frames recorded with [Yarp Data Dumper](https://www.yarp.it/git-master/yarpdatadumper.html) in a folder called `/data` (e.g. `//IITICUBNS010.iit.local/human_pose_estimation/Datasets/HVGA_pilots/01042021/exp2`, download only the `grayscale` folder containing the image frames)

## Usage
- Open a terminal, run the Docker container and compile the pose detector
    ```shell
    $ xhost +
    $ docker run -it -v /abs/path/to/code/dir:/code -v /abs/path/to/data/dir:/data -v /tmp/.X11-unix/:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --runtime=nvidia <image_id>
    # inside the container
    $ cd /code
    $ mkdir build && cd build
    $ cmake .. && make
    ```

- Open a new terminal and run the yarp server in the running container
    ```shell
    $ docker exec -it <container_id/container_name> bash  # executes a bash terminal in the running container
    # inside the container
    $ yarpserver
    ```

- Open a new terminal and run the [Yarp Data Player](https://www.yarp.it/git-master/yarpdataplayer.html) in the running container
    ```shell
    $ docker exec -it <container_id/container_name> bash  # executes a bash terminal in the running container
    # inside the container
    $ yarpdataplayer  # launch data player's gui
    ```

- In the Data Player GUI
    - Select File -> Open Directory, open folder `/data` and click on the `Open` button
    - Select Option -> Repeat
    - Click on the play button

- Run the pose detector in the terminal where it was compiled
    ```shell
    $ ./code/build/test.bin --yarp-image-producer <data_player_port_name> --model-folder /openpose/models
    ```
