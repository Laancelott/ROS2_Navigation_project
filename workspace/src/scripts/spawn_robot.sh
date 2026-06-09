#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity rescue_bot -file /workspace/src/robot_model/rescue_robot.urdf -x 0 -y 0 -z 0.5
