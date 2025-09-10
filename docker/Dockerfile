FROM nvidia/cuda:11.5.2-cudnn8-devel-ubuntu20.04

ARG SOURCE_FOLDER=/usr/local/src
ARG BUILD_TYPE=Release
ARG YCM_VERSION=v0.15.1
ARG YARP_VERSION=v3.7.2
ARG EVENT_DRIVEN_VERSION=master
ARG HPE_VERSION=main

ENV DEBIAN_FRONTEND noninteractive
RUN apt update  
RUN apt install -y build-essential libssl-dev software-properties-common \
    cmake cmake-curses-gui vim nano git sudo openssh-client git \
    libboost-all-dev libmysqlclient-dev ffmpeg libsm6 libxext6 libcanberra-gtk-module \
    apt-transport-https ca-certificates gnupg lsb-core swig \
    libgsl-dev libedit-dev libace-dev libeigen3-dev 
    #qtbase5-dev qt5-default qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls
    
RUN add-apt-repository ppa:deadsnakes/ppa
RUN echo "deb [arch=amd64 trusted=yes] https://apt.prophesee.ai/dists/public/7l58osgr/ubuntu focal essentials" >> /etc/apt/sources.list;

RUN apt update
RUN apt install -y python3.7 python3-pip python3.7-dev
# RUN pip3 install numpy
RUN apt install -y libopencv-dev python3-opencv
RUN apt update && apt install -y metavision-*

RUN cd $SOURCE_FOLDER && \
    git clone --depth 1 --branch $YCM_VERSION https://github.com/robotology/ycm.git &&\
    cd ycm && mkdir build && cd build && \
    cmake .. && make install -j$(nproc)

RUN cd $SOURCE_FOLDER && \
git clone --depth 1  --branch $YARP_VERSION https://github.com/robotology/yarp.git &&\
cd yarp && mkdir build && cd build && \ 
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
        -DYARP_COMPILE_BINDINGS=ON \
        -DCREATE_PYTHON=ON \
        .. && \
make install -j$(nproc)

EXPOSE 10000/tcp 10000/udp

ENV PYTHONPATH $SOURCE_FOLDER/yarp/build/lib/python3:$PYTHONPATH
ENV QT_X11_NO_MITSHM 1

RUN cd $SOURCE_FOLDER && \
    git clone --depth 1 --branch $EVENT_DRIVEN_VERSION https://github.com/robotology/event-driven.git && \
    cd event-driven && mkdir build && cd build && \
    cmake -DVLIB_ENABLE_TS=OFF .. && make install -j$(nproc)

RUN apt update

RUN cd $SOURCE_FOLDER && \
    git clone --depth 1 --branch $HPE_VERSION https://github.com/event-driven-robotics/hpe-core.git &&\
    cd hpe-core/core && mkdir build && cd build && \
    cmake .. && make install -j$(nproc)

RUN python3 -m pip install -r $SOURCE_FOLDER/hpe-core/example/movenet/requirements.txt
ENV PYTHONPATH "${PYTHONPATH}:$SOURCE_FOLDER/hpe-core"
    
RUN cd $SOURCE_FOLDER && \
    git clone --branch main https://github.com/event-driven-robotics/EDPR-APRIL.git && \
    cd EDPR-APRIL && mkdir build && cd build && \
    cmake .. && make install -j$(nproc)

RUN apt install iproute2 -y
RUN apt autoremove && apt clean
RUN rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*

WORKDIR $SOURCE_FOLDER
# COPY run_yarpserver.sh .
# COPY run_atis_bridge.sh .
# COPY run_movenet.sh .
