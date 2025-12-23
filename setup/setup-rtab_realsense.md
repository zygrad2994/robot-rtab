sudo apt update
sudo apt upgrade -y
sudo apt install -y software-properties-common curl wget git build-essential

# X11
sudo nano /etc/ssh/sshd_config
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost no

sudo systemctl restart ssh

ssh -X -Y -i "C:\Users\Admin\.ssh\rpi5_ed25" -o IdentitiesOnly=yes hailo2@192.168.100.14
ssh -i "C:\Users\Admin\.ssh\rpi5_ed25" -o IdentitiesOnly=yes -L 5901:localhost:5901 hailo2@192.168.100.14

# time-fix
sudo apt install -y ntpdate
sudo ntpdate pool.ntp.org

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8 ru_RU ru_RU.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo 'export LANG=en_US.UTF-8' >> ~/.bashrc
echo 'export LC_ALL=en_US.UTF-8' >> ~/.bashrc

# Увеличьте swap перед сборкой

sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

free -h

# 1 Установка ROS 2 Jazzy (desktop)
sudo apt install -y curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install -y ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

echo $ROS_DISTRO

**Тест 1: Запуск узла Python (listener)**

Откройте первую SSH-сессию и выполните:

```bash
ros2 run demo_nodes_py listener
# Ожидаемый вывод:
# [INFO] [listener]: I heard: [Hello World: 0]
# [INFO] [listener]: I heard: [Hello World: 1]
# ... и т.д.
```

**Тест 2: Запуск узла C++ (talker) в отдельной SSH-сессии**

Откройте вторую SSH-сессию и выполните:

```bash
ros2 run demo_nodes_cpp talker
# Ожидаемый вывод:
# [INFO] [talker]: Publishing: 'Hello World: 0'
# [INFO] [talker]: Publishing: 'Hello World: 1'
```

Если вы видите сообщения в обеих сессиях, ROS 2 установлен корректно. Остановите оба узла (Ctrl+C).

# 2 Установка Intel RealSense SDK 2.0 (v2.57.4)

udo apt update
sudo apt upgrade -y
sudo apt install -y git build-essential cmake libusb-1.0-0-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libudev-dev python3-pip python3-dev \
  libssl-dev curl wget
# kernel headers (важно для DKMS/модулей)
sudo apt install -y linux-headers-$(uname -r) || true


TAG=$(curl -s https://api.github.com/repos/IntelRealSense/librealsense/releases/latest | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
echo "Latest librealsense tag: $TAG"

TAG=v2.57.4

# Скачиваем и распаковываем
wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/${TAG}.tar.gz -O librealsense-${TAG}.tar.gz
tar xzf librealsense-${TAG}.tar.gz
cd librealsense-${TAG}

mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=Release
make -j2  # (можно 4)
sudo make install

sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

lsusb | grep -i realsense
# Утилита обнаружения устройств (поставляется с SDK)
rs-enumerate-devices
# Если установлены утилиты librealsense2, можно вызвать:
realsense-viewer  # требует X; если headless — запустите с X11 или VNC

# проверка Python-обвязки
python3 -c "import pyrealsense2 as rs; print(f'Path: {rs.__file__}'); ctx = rs.context(); print(f'Devices: {len(ctx.query_devices())}'); print('SUCCESS')"
Path: /usr/local/lib/python3.12/dist-packages/pyrealsense2/__init__.py
Devices: 1
SUCCESS

# ROS-пакет realsense2_camera для Jazzy
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description

ros2 pkg list | grep realsense2_camera

# 3 Установка RTAB-Map

# рабочее пространство
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
# rtabmap core
git clone https://github.com/introlab/rtabmap.git

# rtabmap_ros — ветка ros2 (официально рекомендовано для ROS2)
git clone -b ros2 https://github.com/introlab/rtabmap_ros.git


# Установить базовые системные зависимости (build tools, OpenCV, PCL, Qt и пр.)
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libopencv-dev libpcl-dev libboost-all-dev \
  qtbase5-dev qtdeclarative5-dev libqt5core5a libqt5gui5 \
  libblas-dev liblapack-dev liblapacke-dev \
  python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool

rosdep update
cd ~/rtabmap_ws
rosdep install --from-paths src --ignore-src -r -y

# Оптимизация сборки для Raspberry Pi 5 
# установить число потоков сборки (первая установка была на 2)
echo 'export MAKEFLAGS="-j2"' >> ~/.bashrc
source ~/.bashrc

# Сборка через colcon
source /opt/ros/jazzy/setup.bash
cd ~/rtabmap_ws

# первая установка была на 2(желательно делать на 1)
colcon build --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --executor parallel --parallel-workers 1

# Подключение сборки и финальное окружение
source ~/ros2_ws/install/setup.bash  
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc

# 4 Настройка окружения и DDS (Cyclone DDS)

# Установка Cyclone DDS RMW-слоя
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp

# Включение RMW_IMPLEMENTATION по умолчанию
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# Можно также задать базовые настройки Cyclone DDS (не обязательно, но полезно на загруженной сети). Пример простого конфига:

cat > ~/.cyclonedds.xml <<'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF
echo 'export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc

# Финальное окружение в .bashrc
# Проверьте, что в ~/.bashrc есть:

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export MAKEFLAGS="-j2"

запуск 
ros2 launch realsense2_camera rs_launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_color_optical_frame (?)
cd ~/ros2_ws

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_color_optical_frame \
  approx_sync:=true \
  use_sim_time:=false \
  rtabmapviz:=true \
  rviz:=true \
  sync_queue_size:=30 \
  topic_queue_size:=30