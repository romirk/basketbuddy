FROM dustynv/ros:humble-ros-core-l4t-r35.4.1

ENV OPENCV_DEB='OpenCV-4.5.0-aarch64.tar.gz'
ENV OPENCV_URL='https://nvidia.box.com/shared/static/2hssa5g3v28ozvo3tc3qwxmn78yerca9.gz'

COPY . /bb

# RUN cd /opt && bash /bb/scripts/opencv_install.sh
RUN apt-get purge -y '.*opencv.*' || echo "previous OpenCV installation not found"

RUN apt update && apt upgrade -y && \
    apt install -y ros-${ROS_DISTRO}-slam-toolbox && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN bash /bb/scripts/setup.sh

CMD ["bash", "/bb/scripts/start.sh"]
