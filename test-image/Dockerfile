FROM ros:foxy-ros-base

RUN apt update

RUN apt -y install ros-dev-tools

RUN apt -y install wget curl nano

COPY entrypoint.sh /

RUN chmod +x /entrypoint.sh

WORKDIR /home/ros2_ws/src

COPY minimal_publisher/ /home/ros2_ws/src/minimal_publisher

WORKDIR /home/ros2_ws

RUN colcon build --packages-select minimal_publisher

ENTRYPOINT [ "/entrypoint.sh" ]
