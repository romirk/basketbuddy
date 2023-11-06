FROM ros:humble-ros-core

RUN apt update && apt upgrade -y && apt install -y \
    ros-$ROS_DISTRO-slam-toolbox && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./rosbot2.yaml /rosbot2.yaml

RUN echo $(dpkg -s ros-$ROS_DISTRO-slam-toolbox | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt
