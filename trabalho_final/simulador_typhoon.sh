#!/bin/bash
echo "Vamos ligar o simulador da PX4"
cd ~/src/Firmware
DONT_RUN=1 make px4_sitl gazebo_typhoon_h480
echo "Ajustando variaveis"
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
echo "Lancando controle por ROS"
roslaunch trabalho_final mavros_posix_sitl_trabalho.launch
