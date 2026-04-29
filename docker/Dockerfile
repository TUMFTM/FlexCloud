ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ARG ROS_DISTRO=jazzy
ARG TZ=Europe/Berlin

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV TZ=${TZ}
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}]: {message}"

# Base utilities & locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    gpg \
    wget && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ROS 2 apt repository
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm -f /tmp/ros2-apt-source.deb

# Build & runtime dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    vim \
    build-essential \
    git \
    cmake \
    make \
    apt-utils \
    libpcl-dev \
    libcgal-dev \
    libclang-dev \
    libatk-bridge2.0 \
    libfontconfig1-dev \
    libfreetype6-dev \
    libglib2.0-dev \
    libgtk-3-dev \
    libssl-dev \
    libxcb-render0-dev \
    libxcb-shape0-dev \
    libxcb-xfixes0-dev \
    libxkbcommon-dev \
    patchelf \
    mesa-vulkan-drivers \
    python3-pip \
    python3-tk \
    python3-venv \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# Rerun viewer
RUN python3 -m venv /ros_ws/rerun
RUN /ros_ws/rerun/bin/pip install rerun_sdk

# GeographicLib (with geoid dataset)
RUN bash -c "wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.7.tar.gz && \
    tar xfpz v2.7.tar.gz && \
    mkdir -p geographiclib-2.7/BUILD && \
    cd geographiclib-2.7/BUILD && cmake .. && make && make install && \
    cd / && rm -rf /geographiclib-2.7 /v2.7.tar.gz"

RUN geographiclib-get-geoids good

WORKDIR /ros_ws

COPY . /ros_ws/src/flexcloud/

# Build with colcon
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --base-path /ros_ws/src/flexcloud --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Source overlay automatically
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /root/.bashrc && \
    echo '. /ros_ws/install/setup.bash' >> /root/.bashrc && \
    printf '#!/bin/bash\nset -e\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. /ros_ws/install/setup.bash\nexec "$@"\n' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
