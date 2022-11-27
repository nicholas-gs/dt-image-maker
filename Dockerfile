FROM ubuntu:20.04 as base

RUN apt update
RUN apt install -y ca-certificates

RUN apt install -y sudo
RUN apt install -y ssh
RUN apt install -y netplan.io

# resizerootfs
RUN apt install -y udev
RUN apt install -y parted

# ifconfig
RUN apt install -y net-tools

# needed by knod-static-nodes to create a list of static device nodes
RUN apt install -y kmod

# Install our resizerootfs service
COPY root/etc/systemd/ /etc/systemd

RUN systemctl enable resizerootfs
RUN systemctl enable ssh
RUN systemctl enable systemd-networkd
RUN systemctl enable setup-resolve

RUN mkdir -p /opt/nvidia/l4t-packages
RUN touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall

COPY root/etc/apt/ /etc/apt
COPY root/usr/share/keyrings /usr/share/keyrings
RUN apt update

# nv-l4t-usb-device-mode
RUN apt install -y bridge-utils

# https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/updating_jetson_and_host.html
RUN apt install -y -o Dpkg::Options::="--force-overwrite" \
    nvidia-l4t-core \
    nvidia-l4t-init \
    nvidia-l4t-bootloader \
    nvidia-l4t-camera \
    nvidia-l4t-initrd \
    nvidia-l4t-xusb-firmware \
    nvidia-l4t-kernel \
    nvidia-l4t-kernel-dtbs \
    nvidia-l4t-kernel-headers \
    nvidia-l4t-cuda \
    jetson-gpio-common \
    python3-jetson-gpio

RUN rm -rf /opt/nvidia/l4t-packages

RUN apt install -y network-manager

RUN apt install -y iputils-ping

RUN apt install -y nano

RUN apt install -y curl

COPY root/ /

# ROS2 Foxy Installation
RUN apt install -y locales

RUN locale-gen en_US en_US.UTF-8

RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN export LANG=en_US.UTF-8

RUN apt install -y software-properties-common

RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update

RUN apt install -y ros-foxy-ros-base python3-argcomplete

RUN apt install -y ros-dev-tools

RUN useradd -ms /bin/bash jetson
RUN echo 'jetson:jetson' | chpasswd

RUN usermod -a -G sudo jetson
