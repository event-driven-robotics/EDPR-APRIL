
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

#get deps
RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3-dev python3-pip git g++ wget make libprotobuf-dev protobuf-compiler libopencv-dev \
libgoogle-glog-dev libboost-all-dev libcaffe-cuda-dev libhdf5-dev libatlas-base-dev nano

#replace cmake as old version has CUDA variable bugs
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.16.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.16.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.16.0-Linux-x86_64/bin:${PATH}"

#get openpose
WORKDIR /openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git .

#build it
WORKDIR /openpose/build
RUN cmake \
# bypass models download (sometimes the CMU server with the trained models does not work, blocking compilation)
# see https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1602#issuecomment-641653411
          -DDOWNLOAD_BODY_25_MODEL=OFF \
          -DDOWNLOAD_BODY_MPI_MODEL=OFF \
          -DDOWNLOAD_HAND_MODEL=OFF \
          -DDOWNLOAD_FACE_MODEL=OFF \
          .. #&& make -j `nproc`

# remove compilation for the Ampere architecture (cuda 10 is not compatible with it, it needs cuda 11)
# see https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1753#issuecomment-792431838
RUN sed -ie 's/set(AMPERE "80 86")/#&/g'  ../cmake/Cuda.cmake && \
    sed -ie 's/set(AMPERE "80 86")/#&/g'  ../3rdparty/caffe/cmake/Cuda.cmake

RUN make -j`nproc` install

# download models from google drive (https://drive.google.com/file/d/1QCSxJZpnWvM00hx49CJ2zky7PWGzpcEh/view)
# see https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1602#issuecomment-641653411
WORKDIR /openpose/tmp
RUN git clone https://github.com/chentinghao/download_google_drive.git && \
    pip3 install requests tqdm && \
    cd download_google_drive && \
    python3 download_gdrive.py 1QCSxJZpnWvM00hx49CJ2zky7PWGzpcEh ../../models.zip
    
# extract downloaded models
WORKDIR /openpose
RUN apt-get install unzip && \
    unzip models.zip && \
    rm models.zip

