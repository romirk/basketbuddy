FROM ros:iron-perception

WORKDIR /bb

COPY ./scripts/install.sh ./install.sh

RUN bash install.sh && rm install.sh

# everything above this line is cached

COPY ./ros_ws/src/basketbuddy src/basketbuddy/

RUN bash <<EOF
source /opt/ros/${ROS_DISTRO}/setup.bash

rosdep install --from-path src/basketbuddy --ignore-src -r -y
colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release --install-base \
    /bb/install --packages-select basketbuddy --symlink-install
EOF

COPY ./scripts/start.sh .

RUN <<EOF
apt remove -y python3-pip python3-rosdep \
    python3-colcon-common-extensions git
SUDO_FORCE_REMOVE=yes apt autoremove -y
apt clean -y
rm -rf /var/lib/apt/lists/* /bb/ros_ws/log /etc/ros/rosdep \
    /bb/ros_ws/src /bb/ros_ws/build
EOF

CMD ["bash", "/bb/start.sh"]
