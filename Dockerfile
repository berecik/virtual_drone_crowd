# Base Image
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools and dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install Rust Toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install colcon-cargo and colcon-ros-cargo
RUN apt-get update && apt-get install -y python3-pip \
    && pip3 install colcon-cargo colcon-ros-cargo \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 Dependencies (for Micro-XRCE-DDS Agent)
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Setup Workspace
WORKDIR /root/sar_swarm_ws
COPY ./sar_swarm_ws/src /root/sar_swarm_ws/src

# Set ROS Domain ID and Source Workspace in Entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/sar_swarm_ws/install/setup.bash ]; then source /root/sar_swarm_ws/install/setup.bash; fi" >> /root/.bashrc

COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
