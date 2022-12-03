FROM nicholasgs/dt-cps:jetson-foxy as base

# Duckietown

RUN apt update

RUN apt install -y v4l-utils

WORKDIR /home/jetson/dt_ws

RUN mkdir -p src

RUN wget https://raw.githubusercontent.com/nicholas-gs/duckietown_ros2_cps/main/main.repos

RUN vcs import src < main.repos

RUN chown -R jetson:jetson /home/jetson/dt_ws && chmod -R 644 /home/jetson/dt_ws

RUN rosdep install --from-paths src --ignore-src --rosdistro foxy -y

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential

RUN echo "source /home/jetson/dt_ws/install/setup.bash" >> /home/jetson/.bashrc
