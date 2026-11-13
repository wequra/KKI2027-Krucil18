FROM ros:jazzy-ros-base

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO} \
    ROS_DOMAIN_ID=42 \
    RCUTILS_COLORIZED_OUTPUT=1

# MAVROS + geographiclib datasets
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    wget \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && rm -f install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages ultralytics

WORKDIR /dagozilla_ws
COPY src ./src
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="v4l2_camera" && \
    colcon build --symlink-install --base-paths src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /dagozilla_ws/install/setup.bash" >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
