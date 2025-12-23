# Комплексная инструкция: RTAB-Map SLAM на Raspberry Pi 5 + RealSense D435iF + Hailo-8L + YDLidar

**Целевая конфигурация:**
- **Платформа:** Raspberry Pi 5 (8GB RAM)
- **ОС:** Ubuntu Server 24.04 LTS (64-bit)
- **ROS:** ROS 2 Jazzy Desktop
- **Камера:** Intel RealSense D435iF (RGB-D + IMU)
- **LiDAR:** YDLidar X3 или YB1 (2D)
- **AI-ускоритель:** Hailo-8L (M.2)
- **SLAM:** RTAB-Map (из исходников)

**Цель:** Создание работающего стека Visual-SLAM с поддержкой 2D LiDAR и возможностью AI-ускорения семантической обработки изображений.

---

## Содержание
1. [Базовая подготовка системы](#1-базовая-подготовка-системы)
2. [Установка и настройка Hailo-8L](#2-установка-и-настройка-hailo-8l)
3. [Установка ROS 2 Jazzy](#3-установка-ros-2-jazzy)
4. [Установка драйверов камеры и LiDAR](#4-установка-драйверов-камеры-и-lidar)
5. [Установка RTAB-Map из исходного кода](#5-установка-rtab-map-из-исходного-кода)
6. [Интеграция Hailo-8L с ROS](#6-интеграция-hailo-8l-с-ros)
7. [Настройка окружения и создание launch-файлов](#7-настройка-окружения-и-создание-launch-файлов)
8. [Запуск и визуализация по SSH](#8-запуск-и-визуализация-по-ssh)
9. [Возможные проблемы и решения](#9-возможные-проблемы-и-решения)

---

## 1. Базовая подготовка системы


### 1.1 Обновление системы и базовые инструменты

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y software-properties-common curl wget git build-essential cmake pkg-config
```

### 1.2 Синхронизация времени

> **Важно:** Корректное время критично для ROS tf2 и синхронизации сенсоров.

```bash
sudo apt install -y ntpdate
sudo ntpdate pool.ntp.org

# Автоматическая синхронизация времени
sudo systemctl enable systemd-timesyncd
sudo systemctl start systemd-timesyncd
```

### 1.3 Настройка локали

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8 ru_RU ru_RU.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo 'export LANG=en_US.UTF-8' >> ~/.bashrc
echo 'export LC_ALL=en_US.UTF-8' >> ~/.bashrc
source ~/.bashrc
```

### 1.4 Настройка swap (4GB)

> **Критично для сборки:** RPi5 с 8GB может испытывать нехватку памяти при компиляции RTAB-Map.

```bash
free -h
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Оптимизация параметров swap(опционально)
sudo tee -a /etc/sysctl.conf > /dev/null <<EOF
vm.swappiness=10
vm.vfs_cache_pressure=50
EOF
sudo sysctl -p

free -h
```

### 1.5 Настройка SSH для удалённой работы

> **Для визуализации:** Необходимо для запуска RViz2 и GUI-приложений.

sudo apt update
sudo apt install tigervnc-standalone-server tigervnc-common openbox dbus-x11 -y

vncpasswd

nano ~/.vnc/xstartup

!/bin/sh

unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
# Запуск D-Bus
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    eval $(dbus-launch --sh-syntax --exit-with-session)
fi
# Минимальный оконный менеджер
exec openbox-session

chmod +x ~/.vnc/xstartup

sudo nano /etc/systemd/system/vncserver@.service

[Unit]
Description=TigerVNC Server for display %i
After=network.target

[Service]
Type=forking
User=ubuntu
Group=ubuntu
WorkingDirectory=/home/ubuntu

# Очистка предыдущей сессии
ExecStartPre=-/usr/bin/vncserver -kill %i
# Запуск VNC
ExecStart=/usr/bin/vncserver %i -geometry 1920x1080 -depth 24 -localhost no
# Остановка VNC
ExecStop=/usr/bin/vncserver -kill %i

# PID файл
PIDFile=/home/ubuntu/.vnc/%H%i.pid

# Ограничения
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target

sudo sed -i "s/ubuntu/$USER/g" /etc/systemd/system/vncserver@.service


sudo systemctl daemon-reload
sudo systemctl enable vncserver@:1.service
sudo systemctl start vncserver@:1.service
sudo systemctl status vncserver@:1.service

# проверка
vncserver -list

**Подключение с Windows:**
```powershell

# Туннелирование для VNC (если используется)
ssh -i "C:\Users\Admin\.ssh\rpi5_ed25" -o IdentitiesOnly=yes -L 5901:localhost:5901 user@<RPi_IP>
```

---

## 2. Установка и настройка Hailo-8L

### 2.1 Конфигурация boot для PCIe Gen 3

```bash
sudo nano /boot/firmware/config.txt
```

Добавить в конец файла:
```
# Hailo-8L PCIe Configuration
dtparam=pciex1
dtparam=pciex1_gen=3

# Fan control (опционально)
dtparam=fan_temp0=60000
dtparam=fan_temp0_hyst=5000
dtparam=fan_temp0_speed=125

# Отключение Bluetooth для освобождения ресурсов
dtoverlay=disable-bt

# GPU memory (минимальное для headless)
gpu_mem=16
```

### 2.2 Обновление прошивки RPi (при необходимости)

```bash
sudo apt install -y rpi-eeprom
sudo rpi-eeprom-update
sudo rpi-eeprom-update -a  # Если требуется обновление

sudo reboot
```

После перезагрузки:
```bash
lspci | grep -i hailo
sudo lspci -vvv | grep -i "LnkSta"
```

**Ожидаемый вывод:** Устройство Hailo с `LnkSta: Speed 8GT/s (ok), Width x1 (ok)`

### 2.3 Сборка драйвера ядра Hailo

```bash
sudo apt install -y build-essential dkms linux-headers-$(uname -r)

cd ~
git clone https://github.com/hailo-ai/hailort-drivers.git
cd hailort-drivers
git checkout hailo8

# Скачивание прошивки
chmod +x download_firmware.sh
./download_firmware.sh

# Установка прошивки
sudo mkdir -p /lib/firmware/hailo
sudo cp hailo8_fw.*.bin /lib/firmware/hailo/hailo8_fw.bin
ls -la /lib/firmware/hailo/

# Сборка модуля
cd linux/pcie
make all
sudo make install

# Настройка для RPi5
echo "options hailo_pci force_desc_page_size=4096" | sudo tee /etc/modprobe.d/hailo_pci.conf
echo "hailo_pci" | sudo tee /etc/modules-load.d/hailo.conf

# UDEV правила
echo 'KERNEL=="hailo*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hailo.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo reboot
```

### 2.4 Проверка драйвера

```bash
lsmod | grep hailo
sudo dmesg | grep -i hailo
ls -la /dev/hailo*
```

### 2.5 Установка HailoRT Runtime

**Скачать с:** https://hailo.ai/developer-zone/software-downloads/
- Файл: `hailort_4.23.0_arm64.deb`

**Копирование на RPi:**
```powershell
# С Windows
scp "C:\Users\Admin\Downloads\hailort_4.23.0_arm64.deb" user@<RPi_IP>:~/
```

**Установка:**
```bash
cd ~
sudo dpkg -i hailort_*_arm64.deb
sudo apt install -f -y

# Проверка
hailortcli fw-control identify
```

**Ожидаемый вывод:**
```
Executing on device: 0000:01:00.0
Identifying board
Firmware Version: 4.23.0
Device Architecture: HAILO8L
```

---

## 3. Установка ROS 2 Jazzy

### 3.1 Установка ROS 2 Jazzy Desktop

```bash
sudo apt install -y curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
# если не работает 
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc" -OutFile "C:\Users\Admin\Downloads\ros.asc"
scp -i "C:\Users\Admin\.ssh\rpi5_ed25" -o IdentitiesOnly=yes "C:\Users\Admin\Downloads\ros.asc" hailo1@192.168.100.10:~/
curl -sSL ~/ros.asc | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

# затем выполнить оставшуюся часть команды  
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo $ROS_DISTRO  # Должно вывести: jazzy
```

### 3.2 Установка инструментов сборки ROS

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# Инициализация rosdep
sudo rosdep init
rosdep update
```

### 3.3 Настройка Cyclone DDS (рекомендовано для RTAB-Map)

> **Источник:** Официальная документация RTAB-Map рекомендует Cyclone DDS для лучшей производительности.

```bash
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp

echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# Создание конфигурации Cyclone DDS
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
```

### 3.4 Тестирование ROS 2

**Терминал 1:**
```bash
ros2 run demo_nodes_cpp talker
```

**Терминал 2:**
```bash
ros2 run demo_nodes_py listener
```

Если видите сообщения — ROS установлен корректно. Остановите узлы (Ctrl+C).

---

## 4. Установка драйверов камеры Intel RealSense D435iF (RGB-D + IMU)

### 4.1 Установка зависимостей

```bash
sudo apt install -y libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev \
  libudev-dev python3-pip python3-dev libssl-dev
sudo apt install -y linux-headers-$(uname -r) || true
```

### 4.2 Сборка librealsense 2.57.4

```bash
cd ~
TAG=v2.57.4

wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/${TAG}.tar.gz -O librealsense-${TAG}.tar.gz
tar xzf librealsense-${TAG}.tar.gz
cd librealsense-${TAG#v}

mkdir build && cd build
cmake .. \
  -DBUILD_EXAMPLES=true \
  -DBUILD_PYTHON_BINDINGS=true \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON

make -j2  # (можно 4 make -j$(nproc))
sudo make install

# Настройка UDEV правил
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4.3 Проверка установки

```bash
lsusb | grep -i realsense
rs-enumerate-devices

# Проверка Python-обвязки
python3 -c "import pyrealsense2 as rs; ctx = rs.context(); print(f'Devices found: {len(ctx.query_devices())}'); print('SUCCESS')"
```

**Ожидаемый вывод:** `Devices found: 1` и `SUCCESS`

### 4.4 Установка ROS-пакета RealSense

```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
ros2 pkg list | grep realsense2_camera
```

### 4.5 Проверка работы драйвера в ROS

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

**В новом терминале:**
```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```

Остановите драйвер (Ctrl+C).

---

## 5 YDLidar X3/YB1 (2D LiDAR)

### 5.1 Установка зависимостей и настройка прав доступа

```bash
sudo apt install -y libudev-dev
sudo usermod -aG dialout $USER
```

> **Важно:** Необходима перезагрузка или повторный вход для применения группы `dialout`.

### 5.2 Сборка YDLidar SDK

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 5.3 Установка драйвера YDLidar для ROS 2

```bash
cd ~
source /opt/ros/jazzy/setup.bash

# Создание рабочего пространства
mkdir -p ~/ydlidar_ws/src
cd ~/ydlidar_ws/src

# Клонирование драйвера (используем ветку humble, совместима с Jazzy)
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git

cd ~/ydlidar_ws
colcon build --symlink-install

source ~/ydlidar_ws/install/setup.bash
```

### 5.4 Настройка UDEV правил

```bash
chmod +x ~/ydlidar_ws/src/ydlidar_ros2_driver/startup/initenv.sh
sudo sh ~/ydlidar_ws/src/ydlidar_ros2_driver/startup/initenv.sh
```

Проверка:
```bash
ls -l /dev/ydlidar /dev/ttyUSB* 2>/dev/null
```

### 5.5 Конфигурация для YDLidar X3

```bash
nano ~/ydlidar_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
```

**Содержимое (оптимизировано для X3):**
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ydlidar
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 115200
    lidar_type: 1                 # TYPE_TRIANGLE
    device_type: 0                # SERIAL
    isSingleChannel: true         # X3 — одноканальный
    intensity: false
    sample_rate: 3                # 3K для X3
    abnormal_check_count: 4
    fixed_resolution: true
    reversion: false
    inverted: false
    auto_reconnect: true
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 8.0
    range_min: 0.10
    frequency: 6.0                # 4-8 Hz (рекомендуется 6)
    invalid_range_is_inf: false
    debug: false
```

### 5.6 Пересборка и проверка

```bash
cd ~/ydlidar_ws
colcon build --symlink-install
source install/setup.bash

# Проверка устройства
groups $USER | grep dialout
ls -l /dev/ydlidar

# Запуск драйвера
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

**В новом терминале:**
```bash
source ~/ydlidar_ws/install/setup.bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Остановите драйвер (Ctrl+C).

---


## 6 Установка RTAB-Map из исходного кода

### 6.1 Установка системных зависимостей

```bash
sudo apt install -y \
  libopencv-dev libpcl-dev libboost-all-dev \
  qtbase5-dev qtdeclarative5-dev libqt5core5a libqt5gui5 \
  libblas-dev liblapack-dev liblapacke-dev \
  libsqlite3-dev libfreenect-dev libusb-1.0-0-dev \
  libopenni2-dev libdc1394-dev libavcodec-dev libavformat-dev \
  libswscale-dev libvtk9-dev libproj-dev libgtsam-dev
```

### 6.2 Создание рабочего пространства

```bash
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
```

### 6.3 Клонирование репозиториев

```bash
# RTAB-Map core (ветка master для последней версии)
git clone https://github.com/introlab/rtabmap.git

# RTAB-Map ROS 2 wrapper (ветка ros2)
git clone -b ros2 https://github.com/introlab/rtabmap_ros.git
```

### 6.4 Установка зависимостей через rosdep

```bash
cd ~/rtabmap_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 6.5 Оптимизация параметров сборки для RPi5

```bash
echo 'export MAKEFLAGS="-j2"' >> ~/.bashrc
source ~/.bashrc
```

> **Примечание:** Используем `-j2` для снижения нагрузки на память.

### 6.6 Сборка RTAB-Map

```bash
cd ~/rtabmap_ws
source /opt/ros/jazzy/setup.bash

colcon build --merge-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_ALICE_VISION=OFF \
    -DWITH_APRILTAG=ON \
    -DWITH_OPENGV=OFF \
  --executor parallel --parallel-workers 1 \
  --event-handlers console_cohesion+ 2>&1 | tee ~/rtabmap_build.log
```
* МЕТКА
> **Важно:** Сборка может занять 1-3 часа на RPi5. Флаг `--parallel-workers 1` предотвращает перегрузку памяти.

### 6.7 Проверка сборки

```bash
source ~/rtabmap_ws/install/setup.bash
ros2 pkg list | grep rtabmap
```

**Ожидаемый вывод:**
```
rtabmap_msgs
rtabmap_conversions
rtabmap_util
rtabmap_slam
rtabmap_viz
rtabmap_rviz_plugins
```

### 6.8 Добавление в .bashrc

```bash
echo 'source ~/rtabmap_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

---

## 7 Интеграция Hailo-8L с ROS

> **Контекст:** Интеграция AI-обработки для семантической сегментации изображений с камеры.

### 7.1 Установка Hailo TAPPAS
  
TAPPAS — GStreamer-фреймворк для видеопотоков Hailo.
### 7.1 Проверка установленных компонентов

```bash
# Проверка HailoRT
hailortcli fw-control identify

# Проверка GStreamer элементов Hailo
gst-inspect-1.0 hailo
gst-inspect-1.0 hailotools
```

**Вывод для `hailo` (базовый плагин — обычно уже установлен):**
```
Plugin Details:
  Name                     hailo
  Description              hailo gstreamer plugin
  Filename                 /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgsthailo.so
  ...
  hailodevicestats, hailonet, synchailonet
  3 features: +-- 3 elements
```

**Вывод для `hailotools` (требуется TAPPAS Core):**
```
Plugin Details:
  Name                     hailotools
  Description              hailo tools plugin
  Filename                 /lib/aarch64-linux-gnu/gstreamer-1.0/libgsthailotools.so
  ...
  hailoaggregator, hailocropper, hailofilter, hailomuxer, hailooverlay ...
```

> ⚠️ **ВАЖНО:** Если `hailotools` не найден (`No such element or plugin 'hailotools'`), вам **необходимо установить TAPPAS Core** (см. раздел 1.2). Без него недоступны критичные элементы:
> - `hailofilter` — постобработка (NMS, декодирование bounding boxes)
> - `hailooverlay` — отрисовка результатов детекции на изображении
> 
> Базовый плагин `hailo` содержит только `hailonet` (сырой inference), который возвращает тензоры без декодирования.

### 7.2 Если TAPPAS Core не установлен — установка вручную

> ⚠️ **Этот шаг ОБЯЗАТЕЛЕН**, если `gst-inspect-1.0 hailotools` возвращает `No such element or plugin 'hailotools'`

**Вариант B: Сборка из исходников**
```bash
sudo apt update
sudo apt install -y \
  rsync ffmpeg x11-utils \
  python3-dev python3-pip python3-setuptools python3-virtualenv \
  python-gi-dev libgirepository1.0-dev \
  gcc-12 g++-12 cmake git libzmq3-dev

# GStreamer зависимости
sudo apt install -y \
  libcairo2-dev libgirepository1.0-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x

# OpenCV
sudo apt install -y libopencv-dev python3-opencv
```

### 7.3 Клонирование и установка TAPPAS

```bash
cd ~
git clone https://github.com/hailo-ai/tappas.git
cd tappas

# Выбор версии совместимой с HailoRT 4.23.0
git checkout v5.1.0

# Проверка системных требований
./check_system_requirements.sh
```
**Установка на RPi5:**

```bash

# Вариант 2: Ubuntu 24.04 на RPi5 (без target-platform!)
./install.sh --skip-hailort
---
### 1.4 Настройка окружения TAPPAS

```bash
# Добавить в ~/.bashrc
echo '' >> ~/.bashrc
echo '# Hailo TAPPAS Environment' >> ~/.bashrc
echo 'export TAPPAS_WORKSPACE=$HOME/tappas' >> ~/.bashrc
echo 'source $TAPPAS_WORKSPACE/scripts/bash_completion.d/_python-argcomplete 2>/dev/null || true' >> ~/.bashrc
echo 'export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1' >> ~/.bashrc

source ~/.bashrc
```

### 7.5 Проверка установки TAPPAS

```bash
# Очистка кэша GStreamer (важно после установки)
rm -f ~/.cache/gstreamer-1.0/registry.aarch64.bin

# Проверка элементов
gst-inspect-1.0 hailotools
gst-inspect-1.0 hailo

# Список доступных элементов
gst-inspect-1.0 | grep hailo
```

### 7.6 Проверка моделей

Модели уже включены в TAPPAS репозиторий:

```bash
# Модели для Hailo-8/8L (h8)
ls ~/tappas/apps/resources/h8/
# yolov5m_wo_spp.hef  yolov8m.hef  ssd_mobilenet_v1.hef
## Часть 2: Установка gst_bridge для ROS 2

**gst_bridge** — пакет ROS 2, предоставляющий GStreamer элементы для подписки и публикации ROS топиков напрямую из GStreamer пайплайнов.

### 2.1 Создание рабочего пространства

```bash
mkdir -p ~/gst_bridge_ws/src
cd ~/gst_bridge_ws/src
```

### 7.7 Клонирование gst_bridge

```bash
# Основная ветка поддерживает ROS 2
git clone https://github.com/BrettRD/ros-gst-bridge.git
cd ros-gst-bridge
git checkout ros2  # или ros2 если есть отдельная ветка
```

### 7.8 Установка зависимостей

```bash
cd ~/gst_bridge_ws
source /opt/ros/jazzy/setup.bash

# Установка ROS зависимостей
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs

# Дополнительные зависимости для audio_msgs (опционально)
# sudo apt install -y ros-jazzy-audio-common-msgs
```

### 7.9 Сборка gst_bridge

```bash
cd ~/gst_bridge_ws
source /opt/ros/jazzy/setup.bash

# сборка audio_msgs 
colcon build --symlink-install --packages-select audio_msgs
# сборка gst_bridge без audio
colcon build --symlink-install --packages-select gst_bridge --cmake-args -DBUILD_AUDIO=OFF


```

### 7.10 Настройка окружения

```bash
source ~/gst_bridge_ws/install/setup.bash

# Настройка GST_PLUGIN_PATH для нахождения элементов
export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$HOME/gst_bridge_ws/install/gst_bridge/lib/gst_bridge

# Добавить в ~/.bashrc
echo '' >> ~/.bashrc
echo '# gst_bridge for ROS 2' >> ~/.bashrc
echo 'source ~/gst_bridge_ws/install/setup.bash' >> ~/.bashrc
echo 'export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$HOME/gst_bridge_ws/install/gst_bridge/lib/gst_bridge' >> ~/.bashrc

source ~/.bashrc
```

### 7.11 Проверка gst_bridge элементов

```bash
# Проверка плагина
gst-inspect-1.0 ~/gst_bridge_ws/install/gst_bridge/lib/gst_bridge/librosgstbridge.so

# Или через GST_PLUGIN_PATH
gst-inspect-1.0 rosimagesrc
gst-inspect-1.0 rosimagesink
```

**Ожидаемый вывод:**
```
Factory Details:
  Rank                     none (0)
  Long-name                rosimagesrc
  Klass                    Source/Video
  Description              a gstreamer source that transports ROS image_msgs over gstreamer
  ...
  ros-topic              : ROS topic to subscribe to
  ros-frame-id           : frame_id of the image message
```
    

### 8.1 Финальная конфигурация ~/.bashrc

```bash
nano ~/.bashrc
```

**Добавить в конец:**
```bash
# Локаль
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml

# Рабочие пространства (порядок важен!)
source ~/rtabmap_ws/install/setup.bash
source ~/ydlidar_ws/install/setup.bash
source ~/hailo_ros_ws/install/setup.bash

# Оптимизация сборки
export MAKEFLAGS="-j2"

# ROS Domain ID (для сетевого взаимодействия)
export ROS_DOMAIN_ID=0
```

```bash
source ~/.bashrc
```

### 8.2 Создание интеграционного launch-файла

```bash
mkdir -p ~/rtabmap_ws/src/rtabmap_launch_custom
cd ~/rtabmap_ws/src/rtabmap_launch_custom
mkdir launch
nano launch/full_slam.launch.py
```

**Содержимое launch-файла:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Параметры
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # RealSense camera (оптимизировано для RPi5)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': '640x360x15',
            'depth_module.depth_profile': '640x360x15',
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',  # copy
            'enable_sync': 'true',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
        }.items()
    )
    
    # YDLidar
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ydlidar_ros2_driver'),
            '/launch/ydlidar_launch.py'
        ])
    )
    
    # Hailo Semantic Node (опционально)
    hailo_node = Node(
        package='hailo_semantic_node',
        executable='semantic_publisher',
        name='hailo_semantic',
        output='screen'
    )
    
    # RTAB-Map SLAM
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'queue_size': 30,
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            'Reg/Force3DoF': 'true',  # 2D SLAM с лидаром
            'Optimizer/GravitySigma': '0.3',
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/depth/image_rect_raw'),
            ('scan', '/scan'),
            ('imu', '/camera/camera/imu'),
        ],
        arguments=['--delete_db_on_start']
    )
    
    # Static transforms
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        realsense_launch,
        ydlidar_launch,
        # hailo_node,  # Раскомментировать при готовности
        base_to_camera,
        base_to_laser,
        rtabmap_slam,
    ])
```

**Создание package.xml:**
```bash
nano package.xml
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rtabmap_launch_custom</name>
  <version>0.0.1</version>
  <description>Custom RTAB-Map launch files</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <exec_depend>realsense2_camera</exec_depend>
  <exec_depend>ydlidar_ros2_driver</exec_depend>
  <exec_depend>rtabmap_slam</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Создание CMakeLists.txt:**
```bash
nano CMakeLists.txt
```

```cmake
cmake_minimum_required(VERSION 3.5)
project(rtabmap_launch_custom)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

**Сборка:**
```bash
cd ~/rtabmap_ws
colcon build --symlink-install --merge-install --packages-select rtabmap_launch_custom
source install/setup.bash
```

---

## 9 Запуск и визуализация по SSH


### 9.1 Проверка отдельных компонентов

#### 9.1.1 RealSense камера

```bash
source ~/.bashrc
ros2 launch realsense2_camera rs_launch.py

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  rgb_camera.color_profile:=640x360x15 \
  depth_module.depth_profile:=640x360x15 \
  align_depth.enable:=true \
  enable_sync:=true \
  enable_infra1:=false \
  enable_infra2:=false
```

**В новом терминале:**
```bash
source ~/.bashrc
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/imu --once  ?
```

#### 9.1.2 YDLidar

```bash
source ~/.bashrc
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

**В новом терминале:**
```bash
source ~/.bashrc
ros2 topic echo /scan --once
ros2 topic hz /scan
```

#### 9.1.3 Проверка трансформаций

```bash
source ~/.bashrc
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link laser_frame

# Добавление статических трансформаций
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link laser_frame
```

### 9.2 Запуск полного SLAM-стека

```bash
source ~/.bashrc
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=base_link \
  visual_odometry:=true \
  approx_sync:=true \
  qos:=2 \
  Vis/MinInliers:=5 \
  Vis/MaxFeatures:=500 \
  Odom/ResetCountdown:=2
```

**Мониторинг в новом терминале:**
```bash
source ~/.bashrc

# Список активных топиков
ros2 topic list

# Частоты публикации
ros2 topic hz /rtabmap/map
ros2 topic hz /rtabmap/grid_map

# Информация об узлах
ros2 node list
ros2 node info /rtabmap
```

### 9.3 Создание конфигурации RViz2

**Создание файла конфигурации:**
```bash
source ~/.bashrc
rviz2

```

### 9.4 Визуализация через RViz2 (по VNC)

**Запуск RViz2 с конфигурацией:**
```bash
source ~/.bashrc
rviz2 -d ~/rviz_configs/rtabmap_slam.rviz
```

**Ручная настройка RViz2 (если без конфига):**
1. **Fixed Frame:** `map` (или `base_link` если карта ещё не создана)
2. Нажать **Add** и добавить:

| Тип | Topic | Описание |
|-----|-------|----------|
| **TF** | — | Системы координат |
| **LaserScan** | `/scan` | Данные лидара (красные точки) |
| **Image** | `/camera/camera/color/image_raw` | RGB с камеры |
| **Map** | `/map` | 2D occupancy grid |
| **PointCloud2** | `/rtabmap/cloud_map` | 3D облако (опционально) |

> **Важно:** Топики камеры имеют namespace `/camera/camera/...`

### 9.4 Запуск GUI RTAB-Map (альтернатива RViz2)

```bash
source ~/.bashrc
ros2 run rtabmap_viz rtabmap_viz
```

> **Примечание:** rtabmap_viz предоставляет более детальную информацию о процессе SLAM (граф петель, keyframes, статистику).

### 9.5 Удалённая визуализация с отдельного ПК

**На Raspberry Pi:**
```bash
export ROS_DOMAIN_ID=0
source ~/.bashrc
ros2 launch rtabmap_launch_custom full_slam.launch.py
```

**На ПК с Ubuntu + ROS Jazzy:**
```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Проверка видимости топиков
ros2 topic list

# Запуск RViz2
rviz2
```

> **Важно:** Убедитесь, что обе машины находятся в одной сети и используют одинаковый `ROS_DOMAIN_ID`.

---

## 10 Возможные проблемы и решения

| **Проблема** | **Симптом / Ошибка** | **Вероятная причина** | **Решение** |
|--------------|---------------------|----------------------|-------------|
| **Нехватка памяти при сборке** | `c++: fatal error: Killed signal terminated program cc1plus` | Недостаточно RAM/swap | Увеличить swap до 8GB: `sudo fallocate -l 8G /swapfile2`, использовать `-j1` |
| **RealSense не обнаружена** | `No RealSense devices were found!` | UDEV правила не применены или камера не подключена | `sudo udevadm trigger`, `lsusb \| grep Intel`, переподключить USB |
| **YDLidar не доступен** | `Error opening serial port /dev/ydlidar` | Нет прав доступа или не применена группа dialout | Перезагрузка после `usermod -aG dialout`, проверить `ls -l /dev/ydlidar` |
| **Hailo не инициализируется** | `Failed to create vdevice` | Драйвер не загружен или прошивка отсутствует | `lsmod \| grep hailo`, `ls /lib/firmware/hailo/`, `sudo modprobe hailo_pci` |
| **Несинхронизированные данные камеры/LiDAR** | RTAB-Map выводит `Could not get transform` | Отсутствуют статические трансформации | Проверить `tf2_echo base_link camera_link`, добавить static_transform_publisher в launch |
| **Низкая производительность SLAM** | FPS < 5, высокая загрузка CPU | Высокое разрешение камеры или не настроен DDS | Снизить разрешение RealSense (`640x480`), проверить `echo $RMW_IMPLEMENTATION` |
| **Ошибка сборки RTAB-Map: `GTSAM not found`** | CMake не находит GTSAM | Не установлена библиотека GTSAM | `sudo apt install libgtsam-dev` или собрать из исходников |
| **RViz2 не запускается по SSH** | `cannot open display` | X11 Forwarding не настроен | Проверить `/etc/ssh/sshd_config`, `echo $DISPLAY` должно быть `localhost:10.0` |
| **Топики ROS не видны с другого ПК** | `ros2 topic list` пустой | Разные `ROS_DOMAIN_ID` или файрволл | Синхронизировать `ROS_DOMAIN_ID`, отключить файрволл: `sudo ufw disable` |
| **RTAB-Map падает при запуске** | `Segmentation fault` | Несовместимость версий библиотек | Пересобрать RTAB-Map: `colcon build --cmake-clean-cache` |
| **Ошибка лицензии Hailo** | `License check failed` | Требуется активация коммерческой лицензии | Для разработки использовать community license или связаться с Hailo |
| **IMU данные не публикуются** | Топик `/camera/imu` пустой | IMU не включен в launch RealSense | Добавить `enable_gyro: true`, `enable_accel: true` в параметры запуска |
| **Loop closure не срабатывает** | Карта дрейфует без замыканий петель | Недостаточно визуальных features | Увеличить `Kp/MaxFeatures`, улучшить освещение, снизить скорость движения |
| **Colcon build зависает** | Процесс сборки не завершается | Исчерпана память или deadlock | `pkill -9 cc1plus`, очистить build: `rm -rf build install log`, использовать `-j1` |

---

## Дополнительные рекомендации

### Оптимизация производительности

1. **Снижение разрешения камеры:**
```bash
hailo1@rpi51:~$ ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  rgb_camera.color_profile:="640,360,15" \
  depth_module.depth_profile:="640,360,15" \
  enable_infra:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  pointcloud.enable:=false \
  align_depth.enable:=false \
  enable_sync:=true
  ```

2. **Настройка параметров RTAB-Map для RPi5:**
   ```python
   'Rtabmap/DetectionRate': '1.0',  # Обработка каждого кадра
   'Kp/MaxFeatures': '200',         # Снижение для экономии CPU
   'Vis/MaxFeatures': '400',
   'RGBD/OptimizeMaxError': '3.0',
   ```

3. **Использование IMU для лучшей одометрии:**
   - Убедитесь, что IMU калибрована
   - Используйте `robot_localization` для fusion IMU + visual odometry

### Запись и воспроизведение данных

**Запись bag-файла:**
```bash
ros2 bag record -o ~/slam_session_01 \
  /camera/color/image_raw \
  /camera/depth/image_rect_raw \
  /camera/color/camera_info \
  /camera/imu \
  /scan \
  /tf /tf_static
```

**Воспроизведение:**
```bash
ros2 bag play ~/slam_session_01 --clock
```

### Калибровка камеры и IMU

Для повышения точности выполните калибровку:
```bash
# Камера
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/color/image_raw

# IMU (требует Kalibr или imu_utils)
```

---

## Cсылки

- **RTAB-Map документация:** https://github.com/introlab/rtabmap_ros
- **RealSense ROS:** https://github.com/IntelRealSense/realsense-ros
- **YDLidar ROS2:** https://github.com/YDLIDAR/ydlidar_ros2_driver
- **Hailo AI:** https://hailo.ai/developer-zone/
- **ROS 2 Jazzy:** https://docs.ros.org/en/jazzy/

---
## depth_profile и color_profile для RealSense

```bash
hailo1@rpi51:~$ ros2 param describe /camera/camera depth_module.depth_profile
1766351700.584585 [0]       ros2: config: //CycloneDDS/Domain/General: 'NetworkInterfaceAddress': deprecated element (file:///home/hailo1/.cyclonedds.xml line 4)
Parameter name: depth_module.depth_profile
  Type: string
  Description: Available options are:
1280x720x15
1280x720x30
1280x720x6
256x144x300
256x144x90
424x240x15
424x240x30
424x240x6
424x240x60
424x240x90
480x270x15
480x270x30
480x270x6
480x270x60
480x270x90
640x360x15
640x360x30
640x360x6
640x360x60
640x360x90
640x480x15
640x480x30
640x480x6
640x480x60
640x480x90
848x100x100
848x100x300
848x480x15
848x480x30
848x480x6
848x480x60
848x480x90
  Constraints:

  hailo1@rpi51:~$ ros2 param describe /camera/camera rgb_camera.color_profile
1766351703.637201 [0]       ros2: config: //CycloneDDS/Domain/General: 'NetworkInterfaceAddress': deprecated element (file:///home/hailo1/.cyclonedds.xml line 4)
Parameter name: rgb_camera.color_profile
  Type: string
  Description: Available options are:
1280x720x15
1280x720x30
1280x720x6
1920x1080x15
1920x1080x30
1920x1080x6
320x180x30
320x180x6
320x180x60
320x240x30
320x240x6
320x240x60
424x240x15
424x240x30
424x240x6
424x240x60
640x360x15
640x360x30
640x360x6
640x360x60
640x480x15
640x480x30
640x480x6
640x480x60
848x480x15
848x480x30
848x480x6
848x480x60
960x540x15
960x540x30
960x540x6
960x540x60
  Constraints: