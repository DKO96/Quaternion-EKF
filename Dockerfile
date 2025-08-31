FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \ 
    clang-tools \
    cmake \ 
    curl \ 
    flex \ 
    gdb \
    gcc \ 
    git \
    gnupg \ 
    g++ \
    libeigen3-dev \
    lsb-release \
    make \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-pip \ 
    python3-rosdep \ 
    python3-vcstool \
    wget \
    vim \ 
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages kconfiglib jinja2 jsonschema pyros-genmsg

# Install Micro-XRCE-DDS-Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --recursive /home/ubuntu/Micro-XRCE-DDS-Agent \
    && cd /home/ubuntu/Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig /usr/local/lib

# Create a non-root user and set permissions
ENV USERNAME=ubuntu
ENV USER_UID=1000
ENV USER_GID=$USER_UID

RUN mkdir /home/$USERNAME/.config \
    && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Setup sudo
RUN apt-get update \ 
    && apt-get install -y sudo \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# Setup work directory
WORKDIR /home/${USERNAME}

# Source ROS in container
COPY bashrc /home/${USERNAME}/.bashrc
CMD ["bash"]

# Confirm user for runtime
USER $USERNAME