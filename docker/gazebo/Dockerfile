FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Исправленный launch-файл для ROS2 Humble
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 launch gazebo_ros gazebo.launch.py"]