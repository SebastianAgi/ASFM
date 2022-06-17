FROM personalroboticsimperial/prl:jetson2004-noetic-zedsdk
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && apt-get install ros-noetic-xacro -y && \
    apt-get install ros-noetic-robot-state-publisher -y

WORKDIR /root/ros_ws/

CMD echo $ROS_MASTER_URI && bash