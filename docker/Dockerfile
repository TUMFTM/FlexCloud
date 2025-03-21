FROM ubuntu:22.04

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
    cmake \
    make \
    apt-utils \
    libpcl-dev \
    libcgal-dev \
    && rm -rf /var/lib/apt/lists/*

# Rerun viewer
RUN pip3 install rerun_sdk

# GeographicLib
RUN bash -c "wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz && \
    tar xfpz v2.3.tar.gz && \
    mkdir -p geographiclib-2.3/BUILD &&\
    cd geographiclib-2.3/BUILD && cmake .. && make && make install && rm -rf /dev_ws/geographiclib-2.3"

# Remove the zip file after download
RUN rm v2.3.tar.gz

# Get geoid dataset from GeographicLib
RUN geographiclib-get-geoids good

WORKDIR /dev_ws

COPY . /dev_ws/src/flexcloud/

# Install flexcloud
RUN /bin/bash -c 'cmake -S /dev_ws/src/flexcloud -B /dev_ws/build && \
    cmake --build /dev_ws/build'

CMD [ "bash" ]