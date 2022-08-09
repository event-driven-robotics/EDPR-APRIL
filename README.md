# EDPR-APRIL
EDPR repository for the European project [APRIL](http://aprilproject.eu/).

## Installation
The software was tested on Ubuntu 20.04.2 LTS with an Nvidia GPU.

- Install the latest [Nvidia driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver)
- Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu)
- Install [Nvidia Docker Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- Download the repository and build the Docker image
    ```shell
    $ cd /path/to/repository/folder
    $ docker build -t op-yarp - < Dockerfile
    ```


## Usage
- Run the Docker container and, inside it, run the pose detector
    ```shell
    $ xhost +
    $ docker run -it -v /tmp/.X11-unix/:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --runtime=nvidia <image_id>
    $ ./launch_pose_detector.sh
    ```
  The ``yarpmanager`` window will appear as shown in the figure below.
  ![image](images/yarpmanager.png)
  The terminal window will show ``yarpserver``'s IP address (``172.17.0.2`` in the figure below). This might be needed 
  for the next step.
  ![image](images/yarpserver_ip.png)

- Open the script ``launch_yarpview.sh`` and check if line ``11`` has the correct IP address of ``yarpserver`` (gathered
  previously, figure below).
  If not, set the correct one.
  ![image](images/yarpview_conf.png)
  
- Run ``yarpview`` on the local machine (yarpview cannot be currently run in the Docker container; this will be fixed in
  a future release)
    ```shell
    $ ./launch_yarpview.sh
    ```
  The script will also download and install locally ``yarp``.
  
- In the ``yarpmanager`` window, open the application menu and select the app "APRIL_WP61a_demo". The list of app 
  components will be shown as in the figure above
 
- Run all components by clicking on the green button ``Run all```.

- ``yarpdataplayer``'s GUI will be shown as in the figure below
  ![image](images/yarpdataplayer.png)
  Select ``File->Open Directory`` and select folder ``data`` as shown in the figure below
  ![image](images/yarpdataplayer_folder.png)

- Two data sources will be used, the raw events (``ATIS``) and the grayscale frames (``yope_grey``)
  ![image](images/yarpdataplayer_data.png)
  Select ``Options->Repeat`` and start playing the recorded data by clicking on the ``Play`` button.

- Finally, connect all components by clicking on the green button ``Connect all`` in ``yarpmanager``'s GUI

The previously opened ``yarpview`` should now show the raw events, the grayscale frame and the overlayed 2D skeleton, as
shown in the figure below.
![image](images/yarpview_gui.png)