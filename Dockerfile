FROM ros:humble-ros-core

ENV OPENCV_DEB='OpenCV-4.5.0-aarch64.tar.gz'
ENV OPENCV_URL='https://nvidia.box.com/shared/static/2hssa5g3v28ozvo3tc3qwxmn78yerca9.gz'

COPY . /bb

# RUN cd /opt && bash /bb/scripts/opencv_install.sh
# RUN apt-get purge -y '.*opencv.*' || echo "previous OpenCV installation not found"

RUN bash /bb/scripts/setup.sh

CMD ["bash", "/bb/scripts/start.sh"]
