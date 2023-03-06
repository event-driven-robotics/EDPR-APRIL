
# base image
FROM nvidia/cuda:11.5.1-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive

# install basic indpendence
RUN apt update
RUN apt install -y build-essential libssl-dev software-properties-common 
RUN apt install -y cmake cmake-curses-gui vim nano git sudo openssh-client git
RUN apt install -y libboost-all-dev libmysqlclient-dev ffmpeg libsm6 libxext6 libcanberra-gtk-module 

##########
# PYTHON & PIP #
##########

# update python
ARG PYTHON_VERSION=3.8
RUN apt install -y python$PYTHON_VERSION python3-pip python3-dev

# create list of alternative python interpreters
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python$PYTHON_VERSION 1 && \
    update-alternatives --config python3 && \
    rm /usr/bin/python3 && \
    ln -s python$PYTHON_VERSION /usr/bin/python3
    
RUN pip3 install numpy

# RUN python3 -m pip install scikit-build matplotlib jupyter pandas

# RUN python3 -m pip install --upgrade pip

#RUN apt install -y python3.7 \
#  python3-tk \

##########
# OPENCV C++ and Python
##########
RUN apt install -y libopencv-dev python3-opencv

#RUN pip3 install opencv-python

# install additional openpose dependencies

#libhdf5-dev libatlas-base-dev


######################
# set github ssh keys #
#######################

ARG ssh_prv_key
ARG ssh_pub_key  

# Authorize SSH Host
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh
RUN ssh-keyscan github.com > /root/.ssh/known_hosts

# Add the keys and set permissions
RUN echo "$ssh_prv_key" > /root/.ssh/id_ed25519 && \
    echo "$ssh_pub_key" > /root/.ssh/id_ed25519.pub && \
    chmod 600 /root/.ssh/id_ed25519 && \
    chmod 600 /root/.ssh/id_ed25519.pub


###############
# NEUROMORHPIC CAMERA DRIVER #
###############
RUN add-apt-repository ppa:deadsnakes/ppa
RUN echo "deb [arch=amd64 trusted=yes] https://apt.prophesee.ai/dists/public/7l58osgr/ubuntu focal essentials" >> /etc/apt/sources.list;
RUN apt update
RUN apt install -y python3.7
RUN apt install -y metavision-*

############
#   INSTALLED FROM SOURCE   #
############

ARG SOURCE_FOLDER=/usr/local/src
ARG BUILD_TYPE=Release

ARG YCM_VERSION=v0.15.1
ARG YARP_VERSION=v3.7.2
ARG EVENT_DRIVEN_VERSION=master
ARG HPE_VERSION=main

RUN apt update

RUN apt install -y \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-core \
    swig

# Install yarp dependencies
RUN apt install -y \
        libgsl-dev \
        libedit-dev \
        libace-dev \
        libeigen3-dev \
# Install QT5 for GUIS
# (NOTE: may be not compatible with nvidia drivers when forwarding screen)
        qtbase5-dev \
        qt5-default \
        qtdeclarative5-dev \
        qtmultimedia5-dev \
        qml-module-qtquick2 \
        qml-module-qtquick-window2 \
        qml-module-qtmultimedia \
        qml-module-qtquick-dialogs \
        qml-module-qtquick-controls

# git clone --depth 1 --branch <branch> url
# Install YCM
RUN cd $SOURCE_FOLDER && \
    git clone --depth 1 --branch $YCM_VERSION https://github.com/robotology/ycm.git &&\
    cd ycm && mkdir build && cd build && \
    cmake .. && make install -j$(nproc)

# Install YARP
RUN cd $SOURCE_FOLDER && \
    git clone --depth 1  --branch $YARP_VERSION https://github.com/robotology/yarp.git &&\
    cd yarp && mkdir build && cd build && \ 
    cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
          -DYARP_COMPILE_BINDINGS=ON \
          -DCREATE_PYTHON=ON \
          .. && \
    make install -j$(nproc)

RUN yarp check
EXPOSE 10000/tcp 10000/udp

# make yarp's python binding visible to python interpreter
ENV PYTHONPATH $SOURCE_FOLDER/yarp/build/lib/python3:$PYTHONPATH

# Some QT-Apps don't show controls without this
ENV QT_X11_NO_MITSHM 1


# install EVENT-DRIVEN
RUN cd $SOURCE_FOLDER && \
    git clone --depth 1 --branch $EVENT_DRIVEN_VERSION https://github.com/robotology/event-driven.git && \
    cd event-driven && mkdir build && cd build && \
    cmake -DVLIB_ENABLE_TS=OFF .. && make install -j$(nproc)
    
# install hpe-core
RUN cd $SOURCE_FOLDER && \
    git clone --depth 1 --branch $HPE_VERSION git@github.com:event-driven-robotics/hpe-core.git &&\
    cd hpe-core/core && mkdir build && cd build && \
    cmake .. && make install -j$(nproc)

# add /usr/local/lib to the library path, so that libcaffe.so compiled with openpose will be used
# instead of the one provided by the nvidia/cuda docker image
#ENV LD_LIBRARY_PATH /usr/local/lib:$LD_LIBRARY_PATH

# install movenet dependencies
RUN python3 -m pip install -r $SOURCE_FOLDER/hpe-core/example/movenet/requirements.txt

ENV PYTHONPATH "${PYTHONPATH}:$SOURCE_FOLDER/hpe-core"
    
#RUN export PYTHONPATH=$PYTHONPATH:$SOURCE_FOLDER/hpe-core
#ENV PYTHONPATH "${PYTHONPATH}:$SOURCE_FOLDER/hpe-core/example/movenet"

# APRIL application
RUN ssh-keyscan github.com > /root/.ssh/known_hosts
RUN cd $SOURCE_FOLDER && \
    git clone --branch main git@github.com:event-driven-robotics/EDPR-APRIL.git && \
    cd EDPR-APRIL && mkdir build && cd build && \
    cmake .. && make install -j$(nproc)
    
RUN apt autoremove && apt clean
RUN rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*

WORKDIR $SOURCE_FOLDER
