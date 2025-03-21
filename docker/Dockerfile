ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    wget \
    curl \
    vim \
    build-essential \
    ca-certificates \
    gnupg2 \
    lsb-release \
    python3-pip \
    python3-tk \
    git \
    yaml-cpp \
    cmake \
    make \
    apt-utils \
    libpcl-dev \
    libcgal-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# GeographicLib
RUN bash -c "wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz && \
    tar xfpz v2.3.tar.gz && \
    mkdir -p geographiclib-2.3/BUILD &&\
    cd geographiclib-2.3/BUILD && cmake .. && make && make install && rm -rf /dev_ws/geographiclib-2.3"

# Remove the zip file after download
RUN rm v2.3.tar.gz

# Get geoid dataset from GeographicLib
RUN geographiclib-get-geoids good

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros_ws

COPY . /ros_ws/src/flexcloud/

# Install flexcloud
RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Set entrypoint by sourcing overlay workspace
RUN echo '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/opt/ros/$ROS_DISTRO/setup.bash"\n. /ros_ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]