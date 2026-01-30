FROM ubuntu:24.04

# Install basic dev tools
RUN apt-get update && apt-get install -y \
    sudo \    
    git \
    build-essential \
    cmake \
    nano \
    locales \
    curl \
    gnupg \
    lsb-release

# Install VSCode
RUN apt-get install wget gpg -y && \
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg && \
    sudo install -D -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/microsoft.gpg && \
    rm -f microsoft.gpg

COPY vscode.sources /etc/apt/sources.list.d/vscode.sources

RUN apt install apt-transport-https -y && \
    apt update && \
    apt install code -y # or code-insiders

# Install ROS2 Jazzy Jalisco
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

RUN update-locale LANG=en_US.UTF-8

RUN sudo sed -i 's/^# deb/deb/' /etc/apt/sources.list

RUN apt-get update && apt-get install -y software-properties-common

RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
noble main" \
> /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install ros-dev-tools -y && apt upgrade -y
RUN apt update && apt install ros-jazzy-desktop -y

# Install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
apt-get update && \
apt-get install gz-harmonic -y

CMD ["/bin/bash"]