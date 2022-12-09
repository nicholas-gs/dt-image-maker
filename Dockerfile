FROM ubuntu:20.04 as base

RUN apt update
RUN apt install -y ca-certificates

RUN apt install -y sudo
RUN apt install -y ssh
RUN apt install -y sshfs
RUN apt install -y netplan.io

# resizerootfs
RUN apt install -y udev
RUN apt install -y parted

# networking
RUN apt install -y net-tools
RUN apt install -y network-manager
RUN apt install -y iputils-ping

RUN apt install -y htop

# i2c
RUN apt install -y i2c-tools

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
    nvidia-l4t-gstreamer \
    nvidia-l4t-jetson-multimedia-api \
    v4l-utils \
    jetson-gpio-common \
    python3-jetson-gpio

RUN rm -rf /opt/nvidia/l4t-packages

# https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Quickstart.html
RUN apt install -y -o Dpkg::Options::="--force-overwrite" \
    libssl1.1 \
    libgstreamer1.0-0 \
    libgstrtspserver-1.0-0 \
    libjansson4 \
    libyaml-cpp-dev

RUN apt install -y -o Dpkg::Options::="--force-overwrite" \
    gstreamer1.0-tools gstreamer1.0-alsa \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

RUN apt install -y -o Dpkg::Options::="--force-overwrite" \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev

# Install desktop environment (not working atm)
# RUN DEBIAN_FRONTEND=noninteractive apt install -y xorg
# RUN DEBIAN_FRONTEND=noninteractive apt install -y lightdm-gtk-greeter \
#     lightdm \
#     openbox

RUN apt install -y nano

RUN apt install -y curl

RUN apt install -y python3-pip

RUN apt install -y libopencv-dev python3-opencv

COPY root/ /

RUN useradd -ms /bin/bash jetson
RUN echo 'jetson:jetson' | chpasswd
RUN usermod -a -G sudo jetson

COPY duckietown/ /home/jetson/duckietown/

RUN chown -R jetson:jetson /home/jetson/duckietown && chmod -R 775 /home/jetson/duckietown

# usb permissions
RUN usermod -aG dialout jetson

# i2c permissions
RUN usermod -aG i2c jetson

# gpio permissions
RUN groupadd -f -r gpio
RUN usermod -a -G gpio jetson

# video permissions
RUN groupadd -f -r video
RUN usermod -a -G video jetson

RUN echo "export ROBOT_CONFIGURATION=\"\$(cat /home/jetson/duckietown/config/robot_configuration)\"" >> /home/jetson/.bashrc && \
    echo "export ROBOT_HARDWARE=\"\$(cat /home/jetson/duckietown/config/robot_hardware)\"" >> /home/jetson/.bashrc && \
    echo "export ROBOT_TYPE=\"\$(cat /home/jetson/duckietown/config/robot_type)\"" >> /home/jetson/.bashrc && \
    echo "export ROBOT_NAME=\"\$(cat /home/jetson/duckietown/config/robot_name)\"" >> /home/jetson/.bashrc && \
    echo "export ROBOT_CALIBRATION_DIR=/home/jetson/duckietown/config/calibrations" >> /home/jetson/.bashrc && \
    echo "export HUT_MCU_ENABLE_PIN=5" >> /home/jetson/.bashrc && \
    echo "/usr/bin/python3 /home/jetson/duckietown/autoboot/enable_mcu.py" >> /home/jetson/.bashrc && \
    echo "export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra\"" >> /home/jetson/.bashrc
