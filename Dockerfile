
# base image
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04


############
# OPENPOSE #
############

# ARG OPENPOSE_VERSION=1.7.0

# install dependencies
RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3-dev python3-pip git g++ wget make libprotobuf-dev protobuf-compiler libopencv-dev \
libgoogle-glog-dev libboost-all-dev libcaffe-cuda-dev libhdf5-dev libatlas-base-dev

# install utils
RUN apt-get install -y unzip nano

# replace cmake, as old version has CUDA variable bugs
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.16.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.16.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.16.0-Linux-x86_64/bin:${PATH}"

#RUN distro=`lsb_release -sr`; \
#    comp=$(awk 'BEGIN{ print "'$distro'"<"'19.04'" }'); \
#    if [ "$comp" -eq 1 ]; then \
#        echo "WARNING: Getting newer CMake version from the kitware repository" ; \
#        wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - ;\
#        apt-add-repository "deb https://apt.kitware.com/ubuntu/ `lsb_release -cs` main";\
#        apt update ;\
#    fi
#RUN apt install -y cmake

# git clone openpose
WORKDIR /openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git . 
    # && git checkout v$OPENPOSE_VERSION

# build openpose
WORKDIR /openpose/build
RUN cmake \
# bypass models download (sometimes the CMU server with the trained models does not work, blocking compilation)
# see https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1602#issuecomment-641653411
          -DDOWNLOAD_BODY_25_MODEL=OFF \
          -DDOWNLOAD_BODY_MPI_MODEL=OFF \
          -DDOWNLOAD_HAND_MODEL=OFF \
          -DDOWNLOAD_FACE_MODEL=OFF \
          ..

# remove compilation for the Ampere architecture (cuda 10 is not compatible with it, it needs cuda 11)
# see https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1753#issuecomment-792431838
RUN sed -ie 's/set(AMPERE "80 86")/#&/g'  ../cmake/Cuda.cmake && \
    sed -ie 's/set(AMPERE "80 86")/#&/g'  ../3rdparty/caffe/cmake/Cuda.cmake

RUN make -j`nproc`
RUN make install


############
#   YARP   #
############

RUN echo "*************** building yarp ****************"

ARG YARP_VERSION=3.4.4
# ARG YCM_VERSION=0.11.1
ARG YCM_VERSION=0.12.1
ARG BUILD_TYPE=Release
ARG SOURCE_FOLDER=/usr/local
ARG OPENGL=0

ENV DEBIAN_FRONTEND noninteractive 

RUN apt update

RUN apt install -y \
    apt-transport-https \
    ca-certificates \
    gnupg \
    software-properties-common \
    lsb-core
    
# Install useful packages
RUN apt install -y \
        build-essential \
        cmake-curses-gui \
        libssl-dev \
        iputils-ping \
        iproute2
        
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
        qml-module-qtquick-controls \
# Setup HW Acceleration for Intel graphic cards
        libgl1-mesa-glx \
        libgl1-mesa-dri \
# Install swig for python bindings
        swig -y \
# Configure virtual environment for python
        && apt-get autoremove \
        && apt-get clean \
        && rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*

RUN python3 -m pip install --upgrade pip setuptools wheel && \
    pip3 install \
    virtualenv \
    virtualenvwrapper

# RUN ln -s /usr/bin/python3 /usr/bin/python && 
ENV VIRTUALENVWRAPPER_PYTHON /usr/bin/python3
RUN echo 'source /usr/local/bin/virtualenvwrapper.sh' | cat - /root/.bashrc > temp && mv temp /root/.bashrc

RUN cd $SOURCE_FOLDER && \
    git clone https://github.com/robotology/ycm.git && \
    cd ycm && \
    git checkout v$YCM_VERSION && \
    mkdir build && cd build && \
    cmake .. && \
    make -j `nproc` install


# Install YARP with GUIS and Python bindings
RUN cd $SOURCE_FOLDER && \
    git clone https://github.com/robotology/yarp.git &&\
    cd yarp &&\
    git checkout v$YARP_VERSION &&\
    mkdir build && cd build &&\
    cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
          -DYARP_COMPILE_BINDINGS=ON \
          -DCREATE_PYTHON=ON \
          -DYARP_USE_PYTHON_VERSION=3 \
          .. &&\
    make -j `nproc` install

RUN PY_VER=`python3 --version | awk '{print $2}' | awk -F "." 'BEGIN { OFS = "." }{print $1,$2}'` && \
    ln -s /usr/local/lib/python3/dist-packages/*yarp* /usr/local/lib/python$PY_VER/dist-packages/

RUN yarp check
EXPOSE 10000/tcp 10000/udp

# Some QT-Apps don't show controls without this
ENV QT_X11_NO_MITSHM 1


# add /usr/local/lib to the library path, so that libcaffe.so compiled with openpose will be used
# instead of the one provided by the nvidia/cuda docker image
ENV LD_LIBRARY_PATH /usr/local/lib:$LD_LIBRARY_PATH


############
# OPENPOSE #
############

# download models from google drive (https://drive.google.com/file/d/1QCSxJZpnWvM00hx49CJ2zky7PWGzpcEh/view)
# see https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1602#issuecomment-641653411
WORKDIR /openpose/tmp
RUN git clone https://github.com/chentinghao/download_google_drive.git && \
    pip3 install requests tqdm && \
    cd download_google_drive && \
    python3 download_gdrive.py 1QCSxJZpnWvM00hx49CJ2zky7PWGzpcEh ../../models.zip

# extract downloaded models
WORKDIR /openpose
RUN unzip models.zip && \
    rm models.zip


#################
# POSE DETECTOR #
#################

# download repository and compile the pose detector
WORKDIR /
RUN git clone https://nicolocarissimi:13579aA!@github.com/event-driven-robotics/EDPR-APRIL.git && cd EDPR-APRIL && \
    git checkout openpose-yarp-docker && \
    cd code && mkdir build && cd build && \
    cmake .. && make

# download demo data
WORKDIR /data
RUN apt-get update && \
    apt-get install curl && \
    curl 'https://istitutoitalianotecnologia.sharepoint.com/sites/EDPR-AVI/_layouts/15/download.aspx?SourceUrl=%2Fsites%2FEDPR%2DAVI%2FDocumenti%20condivisi%2FGeneral%2Fapril%2Fshared%5Fdata%2FAPRIL%5FWP61a%5Fdemo%2Etar%2Egz' -H 'User-Agent: Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:89.0) Gecko/20100101 Firefox/89.0' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' -H 'Accept-Language: en-US,en;q=0.5' --compressed -H 'Referer: https://istitutoitalianotecnologia.sharepoint.com/sites/EDPR-AVI/Documenti%20condivisi/Forms/AllItems.aspx?FolderCTID=0x01200095DA9827815A3D42985CD90A319F3082&id=%2Fsites%2FEDPR%2DAVI%2FDocumenti%20condivisi%2FGeneral%2Fapril%2Fshared%5Fdata%2FAPRIL%5FWP61a%5Fdemo%2Etar%2Egz&parent=%2Fsites%2FEDPR%2DAVI%2FDocumenti%20condivisi%2FGeneral%2Fapril%2Fshared%5Fdata' -H 'Upgrade-Insecure-Requests: 1' -H 'Connection: keep-alive' -H 'Cookie: MicrosoftApplicationsTelemetryDeviceId=77291d76-24c6-3496-9406-095e391e5593; MicrosoftApplicationsTelemetryFirstLaunchTime=1624894261466; rtFa=4DQkO7XqlAvryeNAjJZxd5Ap/DSvdZ7bQ6hpVmqLxYgmQkYyRjQ2NUUtMUY3NS00QjZGLUJFMjktQkY1MzI3M0JDMURBAS/5lAOx/bUkmer7/yj3Hu6n1bH47eUQIWJLa9U9ymS+VwWsC1oACQcbiKH/VgksZ+BkaMzK+hDe338cshqdvKKLwSp4J+YVs1a0BxOQqi8zGDJoQdqOZ7aVg6H5N/+bmp2gYVqguuuEm5j4vgr2tawLXdW4xlvXuDhkInWhjGNHEsNYjpcUuCj9Or0vaYP76kaM5CNx56jlAKrhl3/pTKdAp3uFvZGVBI+oO/q8qr91kPJmqunrgB/doQ02gkW5sSMMBbqwk1iu+gwrYR+xb3w4eRds7Ob5HojFxEd/LT7ONj7S3dvBpL+3xK3YznzF4UIEeK+urYF4aNoif9T/wUUAAAA=; PowerPointWacDataCenter=GEU2; WacDataCenter=GEU2; WSS_FullScreenMode=false; FedAuth=77u/PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0idXRmLTgiPz48U1A+VjEwLDBoLmZ8bWVtYmVyc2hpcHwxMDAzM2ZmZjkwMTIwZWFmQGxpdmUuY29tLDAjLmZ8bWVtYmVyc2hpcHxuaWNvbG8uY2FyaXNzaW1pQGlpdC5pdCwxMzI2NjU5Njk1NjAwMDAwMDAsMTMyNjE2OTEyMjUwMDAwMDAwLDEzMjY5Nzk4OTIwMDY1MDAyMiw5MC4xNDcuMjYuMjM2LDY3LGJmMmY0NjVlLTFmNzUtNGI2Zi1iZTI5LWJmNTMyNzNiYzFkYSwsMmE1MjQ4OTgtYTNjOS00YzgxLWI0YWQtYjU3N2U0MjM0MTk5LDkzNGZkNjlmLWQwOGUtYzAwMC0wZTk1LTMxNWRlOThkMWRkYixkNDU1ZDY5Zi01MDJkLWMwMDAtMzBhYy01ZDQ3ODE0OWZjMjYsLDAsMTMyNjkzNzA1MjAwNjUwMDIyLDEzMjY5NjI2MTIwMDY1MDAyMiwsLGV5SjRiWE5mWTJNaU9pSmJYQ0pEVURGY0lsMGlmUT09LDI2NTA0Njc3NDM5OTk5OTk5OTksMTMyNjkzNjAzNjMwMDAwMDAwLGU4NDgxYzA0LTEyMDktNDE0Mi05YWZlLTUwOWE0ZmYxYTY5ZiwsLCwsLFpQUXlZRnlHdUQ2Rjh3LzFSaThFQlNyL05QYmE3Szg0V3JTbWdXV3M3RVVJalRlNUlkMFk1L3owTEFVRCtjRTdlYXdSWXBGSlg1NGZoNFNQYkFaOW9naWFpTUNaUUNoOVRtUlo0RUZTNnhweC9EU2daWlRzd1pxd3hTRU91YWhMdTU5STQ4eG8wMDZkVFNTTVRISys3VkMrU3lydjMxSVJzRGQ1WWxBYVZacmdwRnQzNlRIbUJ4Z2Jza2lVNE5XMGRlL0ZqdGNuQnc0YkpiSkRMd3NrcmpmWE9qTnNuRkNhRENtb3V2d2pjb0NTUUEvWUFHN0lEWStMYVR4a2lPdzlBcHIydHRlTXNvQmhBT3VkYW1rRE1ZdGRpUkt6RUZIZHFrWFdWQmZOT3BIY1hYaFJvdjBmc3JmQzlzdTFPOTdkemdYY1V6NHoxM0E3OFRiLzNKTmVqUT09PC9TUD4=; CCSInfo=MDEvMDcvMjAyMSAxNTozMTowMjgXirtbUo8DorFTCMC9bcU5hnCD1S8FJUFbAsGBvoHlq5o8KlmZYOVKdRpOV2lWkqFSqryEz0NQg68zeGmSXq1MZb3TCTe1nFfjhwHoinJGqELPs+6CDfriiYdIv0awOTzOsFClvzr4y+nT/LXkLdWgCfwJMXzW+yJSiBYuLvSuz60hCRIHvpTm3w6O7qU8zTkH2P7fJE0QQ+rq4gE5Ouzw0AV65S70IEuxeiK8FTvnS/iMD4LnaiuRYWo5mVKaMLC5RRwNB1XB426HfKL2l4ZOkgUfgkMCX1l55fLOMMwOMdcuFFY6te9k5vfzZdbAFPMWtnwV6CMoOzOrFj/5M7YTAAAA' --output APRIL_WP61a_demo.tar.gz && \
    tar -zxvf APRIL_WP61a_demo.tar.gz

WORKDIR /EDPR-APRIL
