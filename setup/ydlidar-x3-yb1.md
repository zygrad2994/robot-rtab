sudo apt update && sudo apt upgrade -y

# Установка инструментов сборки и зависимостей
sudo apt install -y cmake pkg-config build-essential git

# Установка зависимостей для работы с последовательным портом
sudo apt install -y libudev-dev

# Добавление пользователя в группу dialout для доступа к USB-портам
sudo usermod -aG dialout $USER

# Сборка и установка YDLidar SDK
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
# Обновление кэша библиотек
sudo ldconfig

# Создание рабочего пространства ROS 2 и установка драйвера YDLidar

cd ~
source /opt/ros/jazzy/setup.bash
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver

cd ~/ydlidar_ros2_ws
colcon build --symlink-install
# Проверить и подключить локальный setup для пакета
ls -la ~/ydlidar_ros2_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/local_setup.bash
source ~/ydlidar_ros2_ws/install/ydlidar_ros2_driver/share/ydlidar_ros2_driver/local_setup.bash

# Проверить, доступен ли пакет
ros2 pkg prefix ydlidar_ros2_driver
ros2 pkg list | grep ydlidar

# Сборка с объединённой установкой
colcon build --symlink-install --merge-install --event-handlers console_cohesion+ --verbose 2>&1 | tee ~/colcon_build_ydlidar_merge.log

# После успеха:
source ~/ydlidar_ros2_ws/install/setup.bash

# Установка прав на скрипты
chmod 0777 ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/startup/*

# Запуск скрипта создания udev-правила
sudo sh ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/startup/initenv.sh

# Настройка конфигурационного файла
nano ~/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml

ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ydlidar            # Или /dev/ttyUSB0 если не настроен udev
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 115200              # X3/YB1 использует 115200
    lidar_type: 1                 # TYPE_TRIANGLE для X3
    device_type: 0                # YDLIDAR_TYPE_SERIAL
    isSingleChannel: true         # ВАЖНО: X3 - одноканальный!
    intensity: false              # X3 не поддерживает интенсивность
    intensity_bit: 0
    sample_rate: 3                # X3 использует 3K
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: false              # X3 не требует реверса
    inverted: false
    auto_reconnect: true
    support_motor_dtr: true       # X3 поддерживает DTR
    angle_max: 180.0
    angle_min: -180.0
    range_max: 8.0                # Макс. дальность X3 = 8м
    range_min: 0.10               # Мин. дальность X3 = 0.1м
    frequency: 6.0                # 4-8 Hz для X3 (рекомендуется 6)
    invalid_range_is_inf: false
    debug: false


# Пересоберем после изменений:
cd ~/ydlidar_ros2_ws
colcon build --symlink-install
source install/setup.bash

ls -l /dev/ttyUSB* /dev/ydlidar 2>/dev/null
groups $USER | grep dialout

# Запуск драйвера
source /opt/ros/jazzy/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash
# Запуск
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
# В новом терминале проверьте топик /scan:
source /opt/ros/jazzy/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic hz /scan
# Просмотр данных с помощью RViz2
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py

# Экспорт ROS_DOMAIN_ID (должен совпадать на обеих машинах)
export ROS_DOMAIN_ID=0

На ПК с графикой (клиент):
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
# Запуск RViz2
rviz2