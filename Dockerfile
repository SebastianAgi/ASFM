FROM personalroboticsimperial/prl:jetson2004-noetic-zedsdk
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y

WORKDIR $CATKIN_WS

CMD echo Hello everybody!