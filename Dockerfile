FROM ros:humble-ros-core

RUN apt update && apt upgrade -y && \
    apt install -y git python3 python3-pip \
    libudev-dev ros-${ROS_DISTRO}-slam-toolbox \
    python3-rospkg \
    && apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY . /bb

RUN bash /bb/scripts/setup.sh

CMD ["bash"]
