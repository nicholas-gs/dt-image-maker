FROM nicholasgs/dt-cps:jetson-foxy as base

# Duckietown

RUN apt update

# handling media devices.
RUN apt install -y v4l-utils

# process and system monitoring
RUN pip3 install psutil

USER jetson:jetson

WORKDIR /home/jetson/dt_ws

RUN mkdir -p src

RUN wget https://raw.githubusercontent.com/nicholas-gs/duckietown_ros2_cps/main/main.repos

RUN vcs import src < main.repos

USER root:root

RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y

USER jetson:jetson

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential \
    --parallel-workers 2

RUN echo "source /home/jetson/dt_ws/install/setup.bash" >> /home/jetson/.bashrc
