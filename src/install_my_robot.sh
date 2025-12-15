#!/bin/bash
set -e

echo "1) Обновление системы"
sudo apt update && sudo apt upgrade -y

echo "2) Установка базовых утилит"
sudo apt install -y build-essential git curl wget python3-colcon-common-extensions

echo "3) Установка ROS2 Humble"
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2-latest.list
sudo apt update
sudo apt install -y ros-humble-desktop

echo "4) Настройка окружения ROS2"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

echo "5) rosdep"
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update

echo "6) Создание workspace и клонирование RTAB-Map"
mkdir -p ~/my_robot_ws/src
cd ~/my_robot_ws/src
git clone https://github.com/introlab/rtabmap.git || true
git clone https://github.com/introlab/rtabmap_ros.git || true
mkdir -p my_robot_bringup/launch my_robot_bringup/config

cd ~/my_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo "source ~/my_robot_ws/install/setup.bash" >> ~/.bashrc
echo "Установка завершена!"
