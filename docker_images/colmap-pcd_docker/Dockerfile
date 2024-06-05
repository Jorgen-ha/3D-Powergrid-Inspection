# Credits: https://github.com/colmap/colmap/blob/main/docker/Dockerfile

ARG UBUNTU_VERSION=22.04
ARG NVIDIA_CUDA_VERSION=11.8.0

#
# Docker builder stage.
#
FROM nvcr.io/nvidia/cuda:${NVIDIA_CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

# Prevent stop building ubuntu at time zone selection.
ENV DEBIAN_FRONTEND=noninteractive

# Prepare and empty machine for building.
RUN apt-get update && \
    apt-get install -y --no-install-recommends --no-install-suggests \
        git \
        cmake \
        build-essential \
        libboost-program-options-dev \
        libboost-filesystem-dev \
        libboost-graph-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libsuitesparse-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libglew-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libcgal-dev \
        libcgal-qt5-dev \
        libatlas-base-dev \
        libsuitesparse-dev \
        libpcl-dev \
        libopencv-dev \
        libc6 \
        libceres2 \
        libfreeimage3 \
        libgcc-s1 \
        libgl1 \
        libglew2.2 \
        libqt5core5a \
        libqt5gui5 \
        libqt5widgets5 \
        valgrind \
        wget

# Download and build Ceres Solver
RUN wget -N http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && \
    tar -xvf ceres-solver-2.2.0.tar.gz --no-same-owner && \
    rm ceres-solver-2.2.0.tar.gz
RUN mkdir ceres-bin && cd ceres-bin && \
    cmake ../ceres-solver-2.2.0 && \
    make -j3 && \
    make install

# Build and install COLMAP-PCD.
RUN git clone --recursive https://github.com/XiaoBaiiiiii/colmap-pcd.git &&\
    cd colmap-pcd &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_CXX_STANDARD=20 .. &&\
    make -j &&\
    make install 

# If installing packages for testing purposes, do it here
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends --no-install-suggests \
#     PACKAGE-NAME

WORKDIR /working
