FROM ubuntu:20.04

LABEL maintainer "Rodrigo Chacon <rac17@ic.ac.uk>"

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic
ENV MSCL_LIB_PATH /usr/share/c++-mscl

SHELL ["/bin/bash","-c"]

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup the sources list
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# built-in packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-noetic-ros-base \
    && apt-get autoclean \
    && apt-get autoremove \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    # Basic utilities
    iputils-ping \
    wget \
    # ROS packages
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz \
    build-essential \
    --no-install-recommends \
    && apt-get autoclean \
    && apt-get autoremove \
    # Clear apt-cache to reduce image size
    && rm -rf /var/lib/apt/lists/*

# Spot PRL
RUN apt-get update && apt-get install -y \
    && pip3 install catkin_tools pyyaml rospkg PyQt5 pydot readchar empy defusedxml

# Boston Dynamics API
RUN python3 -m pip install bosdyn-client==3.0.0 bosdyn-mission==3.0.0 bosdyn-choreography-client==3.0.0

# RUN python3 -m pip install --upgrade time 
# protobuf bosdyn-client

WORKDIR /workspace
RUN rosdep init && rosdep update

# Create local catkin ws
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
# Set the working directory to /catkin_ws
WORKDIR $CATKIN_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make -DCMAKE_BUILD_TYPE=Release

CMD ["bash"]