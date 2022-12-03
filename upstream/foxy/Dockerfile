FROM nicholasgs/dt-cps:jetson-base as base

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

RUN rosdep init && rosdep update

RUN echo "source /opt/ros/foxy/setup.sh" >> /home/jetson/.bashrc
