FROM personalroboticsimperial/prl:jetson2004-noetic
SHELL ["/bin/bash", "-c"] 

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    wget \
    git \
    build-essential \
    cmake \
    gcc-8 \
    g++-8 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8

#####################################################################################################
############################################# ZED SDK ###############################################
#####################################################################################################
WORKDIR /zed

COPY .nv_tegra_release /etc/nv_tegra_release

RUN wget -q -O ZED_SDK_Jetson_326.run https://download.stereolabs.com/zedsdk/3.7/l4t32.6/jetsons
RUN chmod +x ZED_SDK_Jetson_326.run && \
    DEBIAN_FRONTEND=noninteractive ./ZED_SDK_Jetson_326.run -- silent skip_tools && \
    rm ZED_SDK_Jetson_326.run

RUN apt-get update -y && apt-get install --no-install-recommends python3-pip -y && \
    wget download.stereolabs.com/zedsdk/pyzed -O /usr/local/zed/get_python_api.py &&  \
    python3 /usr/local/zed/get_python_api.py && \
    python3 -m pip install numpy opencv-python *.whl && \
    rm *.whl

#####################################################################################################
############################################ ROS PACKAGE ############################################
#####################################################################################################

RUN mkdir -p /root/ros_ws/src/

WORKDIR /root/ros_ws/

#RUN cd src && git clone --recurse-submodules -j8 https://github.com/stereolabs/zed-ros-wrapper.git

RUN apt update && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src -r -y && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install opencv-contrib-python

RUN apt-get update -y && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-diagnostic-updater \
    libusb-1.0-0-dev \
    ros-noetic-xacro \
    ros-robot-state-publisher

RUN source /opt/ros/noetic/setup.bash && \
    catkin config --cmake-args -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"

CMD echo $ROS_MASTER_URI && bash