FROM nvidia/cuda:11.1-cudnn8-devel-ubuntu20.04

# Install dependencies
# ENV TZ=Europe/London
ENV DEBIAN_FRONTEND=noninteractive
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y libglib2.0-0 \
    ffmpeg \
    libsm6 \
    libxext6 \
    libxrender-dev \
    wget \
    git \
    unzip \
    curl \
    nano \
    libyaml-dev \
    locales \
    python3-setuptools \
    python3-pip \
    lsb-core \
    && apt-get -y clean all \
    && rm -rf /var/lib/apt/lists/*

## ROS - noetic stuff
RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    apt install -y ros-noetic-ros-base \
    libeigen3-dev \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-cv-camera \
    ros-noetic-video-stream-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV (not necessary since we are using ROS noetic's OpenCV installation)
# RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
#     apt-get install build-essential -y && \
#     apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y && \
#     mkdir -p /opencv && \
#     cd /opencv && \
#     wget -c https://github.com/opencv/opencv/archive/4.3.0.zip && \
#     unzip 4.3.0.zip && cd opencv-4.3.0 && \
#     mkdir build && cd build &&\
#     cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
#     make -j`nproc` && \
#     make install

# USER developer
RUN useradd -ms /bin/bash developer && echo "developer:developer" | chpasswd && adduser developer sudo && adduser developer root
WORKDIR /home/developer/object_detectio/catkin_ws