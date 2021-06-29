
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
        libssl-dev
        
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
          -DYARP_COMPILE_BINDINGS=OFF \
          -DCREATE_PYTHON=OFF \
          .. &&\
    make -j `nproc` install

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
    apt-get install -y curl && \
    curl 'https://istitutoitalianotecnologia.sharepoint.com/sites/EDPR-AVI/Documenti%20condivisi/General/april/shared_data/APRIL_WP61a_demo.tar.gz' -H 'User-Agent: Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:89.0) Gecko/20100101 Firefox/89.0' -H 'Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8' -H 'Accept-Language: en-US,en;q=0.5' --compressed -H 'Upgrade-Insecure-Requests: 1' -H 'Connection: keep-alive' -H 'Cookie: MicrosoftApplicationsTelemetryDeviceId=ae89db69-c7f1-4a28-abf4-bed3a456261d; MicrosoftApplicationsTelemetryFirstLaunchTime=2021-06-28T15:33:46.804Z; rtFa=YXgqps3MUGIxrje4n8C4Wl7ZcABeCWjzwjwymgmXuocmQkYyRjQ2NUUtMUY3NS00QjZGLUJFMjktQkY1MzI3M0JDMURBPrLSSMVyNnPkXzFNkdMN7N79qjoS+nt+/Al2uApysw87j7jGbRPdNyuWSg3U2es4wG6+5Td3n5SQVq/aYBQDKI313tebn3JgwQnkssDXJ815soYWWrfDUZ/fkrMfsHSKdW1JPXbKS6VCziltgsgQpxLIqMhA4D+w6dutSdiXINPdSJ8RxOCZgeiRBSE+G0Cev4t0NqgBeNf2dmxfr2ShSvsWzu5YEXisKWxtUAkynMm0EinekoRIhjVoGvmxES4ko/YYnMuTdLQV45UpW8Osf/P3wFXOEz5evHTKFHqq+53j5wVbWUorzDjQQtITlH8Eex1IZKCSMUMFLOkNmzxAGEUAAAA=; PowerPointWacDataCenter=GEU2; WacDataCenter=GEU2; WSS_FullScreenMode=false; CCSInfo=MDEvMDcvMjAyMSAxNTozMTowMjgXirtbUo8DorFTCMC9bcU5hnCD1S8FJUFbAsGBvoHlq5o8KlmZYOVKdRpOV2lWkqFSqryEz0NQg68zeGmSXq1MZb3TCTe1nFfjhwHoinJGqELPs+6CDfriiYdIv0awOTzOsFClvzr4y+nT/LXkLdWgCfwJMXzW+yJSiBYuLvSuz60hCRIHvpTm3w6O7qU8zTkH2P7fJE0QQ+rq4gE5Ouzw0AV65S70IEuxeiK8FTvnS/iMD4LnaiuRYWo5mVKaMLC5RRwNB1XB426HfKL2l4ZOkgUfgkMCX1l55fLOMMwOMdcuFFY6te9k5vfzZdbAFPMWtnwV6CMoOzOrFj/5M7YTAAAA; FedAuth=77u/PD94bWwgdmVyc2lvbj0iMS4wIiBlbmNvZGluZz0idXRmLTgiPz48U1A+VjEwLDBoLmZ8bWVtYmVyc2hpcHwxMDAzM2ZmZjkwMTIwZWFmQGxpdmUuY29tLDAjLmZ8bWVtYmVyc2hpcHxuaWNvbG8uY2FyaXNzaW1pQGlpdC5pdCwxMzI2OTQzMDc4OTAwMDAwMDAsMTMyNjkzNjk3NzcwMDAwMDAwLDEzMjY5ODYyNzkyMDU0NzM5NCw5MC4xNDcuMjYuMjM2LDMsYmYyZjQ2NWUtMWY3NS00YjZmLWJlMjktYmY1MzI3M2JjMWRhLCxmOWFlOGMzNC01YjJjLTRiZTQtYjM1ZC01YWU1MTc0NTkwODUsYmQ5MmQ2OWYtODBmMy1jMDAwLTMwNzYtMGYyNGZjYWM4YzI4LGJkOTJkNjlmLTgwZjMtYzAwMC0zMDc2LTBmMjRmY2FjOGMyOCwsMCwxMzI2OTQzNDM5MjAyMzUxMDIsMTMyNjk2ODk5OTIwMjM1MTAyLCwsZXlKNGJYTmZZMk1pT2lKYlhDSkRVREZjSWwwaWZRPT0sMjY1MDQ2Nzc0Mzk5OTk5OTk5OSwxMzI2OTQzMDc5MTAwMDAwMDAsZTg0ODFjMDQtMTIwOS00MTQyLTlhZmUtNTA5YTRmZjFhNjlmLCwsLCwsZFpTZW16QkVoR1UxZEwvY3R6RTRzZ3ovcTVhemtieE5wazJvT0F5TnMyMFh6b0xXeXVBKzZsNVlmYWpxUjk5ODVkcTVzeVJMWEh1aGlJcnJQYUlnMnlGd1Joa25DL05PSFBBT1l1Qm43V09GSjRjQlVnakZHTHZoWnM1S2gvL0VKaEo4clZ2MUtCaThKK0NOc0Q1RFRHNjViWjZyRXVxeitMejQxRjBSMDBmQk5hUXFHWEJmZHBGc3dERDN3eWcwbHhQWTB0em9mbEdBd3BLTEg0M1M1b0NoMlZYd011djR0cGlNWTFoelNNYmxHL0Jiek5FaURybXFXN29Idm96bnh1UXk0RHdkNCtwcUR5WjBEZEpVa2oyUFdjNnozTkhWUTU1VXZ5aXJZQTZPZXVxSno3S1N6Ulg5LzNMdElZR0pwei9NU3JIUjgySEE4Ujg1L085ZjJ3PT08L1NQPg==' -o APRIL_WP61a_demo.tar.gz
RUN tar -zxvf APRIL_WP61a_demo.tar.gz

WORKDIR /EDPR-APRIL
