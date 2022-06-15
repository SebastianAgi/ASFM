FROM personalroboticsimperial/prl:jetson2004-noetic-zedsdk
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y

CMD echo Hello everybody!