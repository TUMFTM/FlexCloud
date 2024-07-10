FROM ubuntu:22.04

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common locales && \
    add-apt-repository universe
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create a non-root user
ARG USERNAME=tum
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

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
    cmake \
    make \
    apt-utils \
    libpcl-dev \
    libcgal-dev \
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

# ROS 2
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y -q --no-install-recommends \ 
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-rviz2

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros_ws

COPY . /ros_ws/src/flexcloud/

# RUN pip install --no-cache-dir --upgrade pip && \
#   pip install --no-cache-dir -r src/multi_lidar_calibration/requirements.txt

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to flexcloud'

CMD [ "bash" ]