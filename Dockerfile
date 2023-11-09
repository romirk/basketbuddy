FROM dustynv/ros:humble-ros-core-l4t-r35.4.1

COPY . /bb

RUN bash /bb/scripts/openvc_install.sh

RUN apt update && apt upgrade -y && \
    apt install -y ros-${ROS_DISTRO}-slam-toolbox && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*



RUN bash /bb/scripts/setup.sh
