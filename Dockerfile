FROM personalroboticsimperial/prl:jetson2004-noetic-zedsdk
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y

WORKDIR $ros_ws

CMD echo Hello everybody!