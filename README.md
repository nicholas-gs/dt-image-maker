# Install ROS2 Foxy on Jetson Nano

## Base Image

Follow the guide [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write-linux) and flash an SD card Jetson Nano Developer Kit SD Card Image. Tested on [JetPack 4.6.1 (L4T 32.7.1)](https://developer.nvidia.com/embedded/jetpack-sdk-461).

## Predependencies

### Update system

Doing an upgrade on Nvidia packages seems to break the bootloader. So avoid upgrading them for the moment.

```bash
sudo apt-mark hold 'nvidia-l4t-*'
sudo apt update
sudo apt upgrade -y
```

### Other necessary packages

Install the following packages.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt install -y \
    build-essential \
    checkinstall \
    libssl-dev

sudo apt install -y --no-install-recommends \
    curl \
    wget \
    gnupg2 \
    lsb-release

sudo rm -rf /var/lib/apt/lists/*
```

The existing CMake version is 3.10 and is too old for building certain ROS2 Foxy
packages. So we need to get a newer version. The recommended way is to get the precompiled binaries.

```bash
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0-Linux-aarch64.tar.gz
tar -zxvf cmake-3.20.0-Linux-aarch64.tar.gz
cd cmake-3.20.0-Linux-aarch64/
sudo cp -rf bin/ doc/ share/ /usr/local/
sudo cp -rf man/* /usr/local/man
sync
# check if version is now 3.20
cmake --version
```

Or if you want to build from source,

```bash
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz
tar -zvxf cmake-3.20.0.tar.gz
cd ~/cmake-3.20.0
./bootstrap
make -j2
sudo checkinstall --pkgname=cmake --pkgversion="3.20-custom" --default
hash -r
cd ~
```

We also need `yaml-cpp-0.6` for some ROS2 Foxy packages. But it is not in the apt repositories so we need to build it from source.

```bash
cd ~
git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6
cd yaml-cpp-0.6
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make -j2
# Copy built binaries
sudo cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/
# Make a soft link
sudo ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6
```

## ROS2

We are now ready to built ROS2 Foxy.

### Set locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add the ROS 2 apt repository

```bash
cd ~
wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
sudo apt-key add ros.asc
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

### Install development tools and ROS tools

```bash
sudo apt update
# install development packages
sudo apt install -y --no-install-recommends \
        build-essential \
        git \
        libbullet-dev \
        libpython3-dev \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-pip \
        python3-pytest-cov \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        python3-rosinstall-generator

# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

sudo rm -rf /var/lib/apt/lists/*
```

```bash
# install some pip packages needed for testing
python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest
```

### Get ROS2 Code

```bash
mkdir -p ~/ros2_build_root/src
cd ~/ros2_build_root

# Download all the necessary repositories
sudo sh -c "rosinstall_generator --deps --rosdistro foxy ros_base launch_xml launch_yaml example_interfaces > ros2.foxy.ros_base.rosinstall && \
cat ros2.foxy.ros_base.rosinstall && \
    vcs import src < ros2.foxy.ros_base.rosinstall"
```

```bash
cd ~/ros2_build_root
# download unreleased packages
sudo sh -c "git clone --branch ros2 https://github.com/Kukanani/vision_msgs src/vision_msgs && \
    git clone --branch foxy https://github.com/ros2/demos demos && \
    cp -r demos/demo_nodes_cpp src/ && \
    cp -r demos/demo_nodes_py src/ && \
    rm -r -f demos"
```

### Install dependencies using rosdep

```bash
sudo apt update
cd ~/ros2_build_root
sudo rosdep init
rosdep update
# install missing ros package dependencies
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers qt_gui"

sudo rm -rf /var/lib/apt/lists/*
```

### Build it!

This process took more than 2 hours and max out the CPU.

```bash
cd ~/ros_build_root
sudo mkdir -p /opt/ros/foxy
# sudo required to write build logs
sudo colcon build --merge-install --install-base /opt/ros/foxy
# We do this twice to make sure everything gets built
# For some reason, this has been an issue
sudo colcon build --merge-install --install-base /opt/ros/foxy

# expand environment variables
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc

source ~/.bashrc
```

### Test

```bash
source ~/.bashrc

# You should see a `chatter` topic being published after
# running this line
ros2 run demo_nodes_py talker
```

## Credits

*  [Official ROS guide](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html).

* https://github.com/dusty-nv/jetson-containers

* https://forums.developer.nvidia.com/t/having-problems-updating-cmake-on-xavier-nx/169265/2

* https://github.com/jetsonhacks/installROS2
