
#!/bin/bash
set -euo pipefail

# ---------- Настройки ----------
USERNAME="${SUDO_USER:-$(whoami)}"
MAP_DIR="/home/${USERNAME}/maps/$(date +%Y-%m-%d_%H-%M-%S)"
LOG_FILE="${MAP_DIR}/slam.log"

RTABMAP_CONFIG="/home/${USERNAME}/.rtabmap/rtabmap_robot.ini"

SENSOR_START_TIMEOUT=10
SLAM_START_TIMEOUT=10
TOPIC_CHECK_INTERVAL=2

mkdir -p "$MAP_DIR"

# ---------- Логирование ----------
mkdir -p "$(dirname "$LOG_FILE")"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "[+] Логирование включено: $LOG_FILE"
echo "[+] Время запуска: $(date)"

# Проверка lsb_release
if command -v lsb_release >/dev/null 2>&1; then
    echo "[+] Система: $(lsb_release -d | cut -f2)"
else
    echo "[!] lsb_release не найден — пропускаю вывод информации о системе"
fi

echo "[+] Пользователь: $USERNAME"
echo "[+] Целевая директория: $MAP_DIR"

# ---------- Функция корректной остановки ----------
cleanup() {
    echo
    echo "[+] Остановка всех ROS2-процессов..."
    pkill -f "realsense2_camera" 2>/dev/null || true
    pkill -f "ydlidar_ros2_driver" 2>/dev/null || true
    pkill -f "rtabmap" 2>/dev/null || true
    pkill -f "rtabmap_ros" 2>/dev/null || true
    sleep 2
    pkill -9 -f "ros2" 2>/dev/null || true
    echo "[+] Все процессы остановлены"
    echo "[+] Карта сохранена в: $MAP_DIR"

    if [[ -f "${MAP_DIR}/rtabmap.db" ]]; then
        DB_SIZE=$(du -h "${MAP_DIR}/rtabmap.db" 2>/dev/null | cut -f1)
        echo "[+] Размер базы данных: $DB_SIZE"
        echo "[+] Количество файлов в директории: $(find "$MAP_DIR" -type f | wc -l)"
    fi
}

trap cleanup INT TERM

# ---------- Проверка системных ресурсов ----------
echo "[+] Проверка системных ресурсов..."

RAM_TOTAL=$(free -m | awk 'NR==2{printf "%.1f", $2/1024}')
RAM_USED=$(free -m | awk 'NR==2{printf "%.1f", $3/1024}')

echo "[+] Доступная память: ${RAM_USED}G / ${RAM_TOTAL}G"

if command -v bc >/dev/null 2>&1; then
    if (( $(echo "$RAM_TOTAL < 7.0" | bc -l) )); then
        echo "[!] Внимание: Мало оперативной памяти для SLAM (рекомендуется >7GB)"
    fi
else
    echo "[!] bc не найден — пропускаю дополнительную проверку RAM"
fi

# Проверка диска
DISK_SPACE_RAW=$(df -BG "$MAP_DIR" | awk 'NR==2 {print $4}')  # например "20G"
DISK_SPACE=${DISK_SPACE_RAW%G}

echo "[+] Свободное место на диске: ${DISK_SPACE}G"

if command -v bc >/dev/null 2>&1; then
    if (( $(echo "$DISK_SPACE < 2" | bc -l) )); then
        echo "[-] Недостаточно места на диске (минимум 2GB)"
        exit 1
    fi
else
    if [[ "$DISK_SPACE" -lt 2 ]]; then
        echo "[-] Недостаточно места на диске (минимум 2GB)"
        exit 1
    fi
fi

# ---------- Инициализация ROS2 (Jazzy / rolling) ----------
ROS_DISTROS=("jazzy" "rolling")
ROS_SETUP_FILE=""

for distro in "${ROS_DISTROS[@]}"; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
        ROS_SETUP_FILE="/opt/ros/${distro}/setup.bash"
        echo "[+] Найдено ROS2: ${distro}"
        break
    fi
done

if [ -z "$ROS_SETUP_FILE" ]; then
    echo "[-] ROS2 Jazzy/Rolling не найден. Установите ROS2 и повторите."
    exit 1
fi

# shellcheck source=/dev/null
set +u
source "$ROS_SETUP_FILE"
set -u
# ---------- Проверка конфигурации RTAB-Map ----------
if [ ! -f "$RTABMAP_CONFIG" ]; then
    echo "[-] Конфиг RTAB-Map не найден: $RTABMAP_CONFIG"
    echo "[+] Создаю базовый конфиг, оптимизированный под Raspberry Pi + D435i..."
    mkdir -p "$(dirname "$RTABMAP_CONFIG")"
    cat > "$RTABMAP_CONFIG" << 'EOF'
# Базовый конфиг RTAB-Map под D435i + 2D SLAM

# Частота обработки
RGB/DetectionRate=10

# Фичи и визуальная одометрия
Kp/MaxFeatures=400
Kp/DetectorStrategy=6
Vis/MaxFeatures=400
Vis/MinInliers=8
Vis/MaxDepth=4.0
Vis/InlierDistance=0.1

# Глубина
Depth/MaxDepth=10.0
Depth/MinDepth=0.3

# 2D SLAM
Optimizer/Slam2D=true
Optimizer/Strategy=1
Reg/Force3DoF=true

# Память и время
Rtabmap/TimeThr=700
Rtabmap/MemThr=0
Rtabmap/PublishRAM=1
Rtabmap/WorkingDirectory=/tmp
Rtabmap/CreateIntermediateNodes=false

# Одометрия
Odom/Strategy=1
Odom/ResetCountdown=1
Odom/GuessMotion=true
Odom/FilteringStrategy=1
Odom/FillInfoData=true
Odom/ImageDecimation=2
EOF
    echo "[+] Базовый конфиг создан: $RTABMAP_CONFIG"
else
    echo "[+] Используется существующий конфиг RTAB-Map: $RTABMAP_CONFIG"
fi

echo "[+] Запуск ручного SLAM-картирования"
echo "[+] Карта будет сохранена в: $MAP_DIR"

# ---------- Проверка сенсоров ----------
echo "[+] Проверка подключения сенсоров..."

if lsusb | grep -qi "Intel.*RealSense"; then
    echo "[+] RealSense D435i обнаружен"
else
    echo "[!] RealSense D435i не найден в lsusb"
fi

if ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | grep -qi "ydlidar\|lidar\|usb"; then
    echo "[+] Лидар обнаружен (ttyUSB/ttyACM присутствуют)"
else
    echo "[!] Лидар в /dev не найден (возможен автоподъём драйвером позже)"
fi

#echo "[+] Проверка USB режима RealSense..."
####fi

# ---------- 1. Запуск RealSense ----------
echo "[+] Запуск RealSense D435i..."
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=false \
    enable_gyro:=false \
    enable_accel:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    pointcloud.enable:=false \
    rgb_camera.color_profile:=640x360x15 \
    depth_module.depth_profile:=640x3x15 \
    enable_sync:=true \
    initial_reset:=true &

sleep 10

echo "[+] Ожидание готовности RealSense (RGB)..."
for ((i=1; i<=SENSOR_START_TIMEOUT; i++)); do
    if ros2 topic list | grep -q "/camera/camera/color/image_raw"; then
        echo "[+] RealSense RGB готов (итерация $i)"
        break
    fi
    sleep "$TOPIC_CHECK_INTERVAL"
    if (( i == SENSOR_START_TIMEOUT )); then
        echo "[-] Топик /camera/camera/color/image_raw не появился"
        cleanup
        exit 1
    fi
done

echo "[+] Проверка наличия depth-данных..."
# При align_depth.enable:=false RealSense публикует необработанный depth
# поэтому проверяем /camera/camera/depth/image_rect_raw
if ! ros2 topic echo /camera/camera/depth/image_rect_raw --once >/dev/null 2>&1; then
    echo "[-] Depth не поступает с /camera/camera/depth/image_rect_raw! SLAM невозможен."
    cleanup
    exit 1
fi
echo "[+] Depth OK"

# ---------- 2. Запуск YDLIDAR ----------
echo "[+] Запуск YDLIDAR X3..."
ros2 launch ydlidar_ros2_driver ydlidar_launch.py &

sleep 5

echo "[+] Ожидание готовности YDLIDAR..."
for ((i=1; i<=SENSOR_START_TIMEOUT; i++)); do
    if ros2 topic list | grep -q "/scan"; then
        echo "[+] YDLIDAR готов (итерация $i)"
        break
    fi
    sleep "$TOPIC_CHECK_INTERVAL"
    if (( i == SENSOR_START_TIMEOUT )); then
        echo "[-] Топик /scan не появился"
        cleanup
        exit 1
    fi
done
# ---------- Создание статических трансформаций ----------
echo "[+] Добавление статических трансформаций..."
# laser: 0 m вперед, 5 cm вверх
ros2 run tf2_ros static_transform_publisher 0 0 0.05 0 0 0 base_link laser_frame &
# camera: 7.5 cm вперед, 2 cm вверх (3 cm ниже лидара)
# Публикуем в frame, который использует RealSense (`camera_color_optical_frame`)
ros2 run tf2_ros static_transform_publisher 0.075 0 0.02 0 0 0 base_link camera_color_optical_frame &
sleep 5
echo "[+] Трансформации созданы"

# ---------- 3. Запуск RTAB-Map SLAM ----------
# Установим DISPLAY/XAUTHORITY для GUI-приложений (VNC)
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=":1"
    echo "[+] Установлен DISPLAY=${DISPLAY}"
else
    echo "[+] DISPLAY уже задан: ${DISPLAY}"
fi
if [ -z "${XAUTHORITY:-}" ] && [ -f "/home/${USERNAME}/.Xauthority" ]; then
    export XAUTHORITY="/home/${USERNAME}/.Xauthority"
    echo "[+] Установлен XAUTHORITY=${XAUTHORITY}"
fi

echo "[+] Запуск RTAB-Map SLAM..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start --params ${RTABMAP_CONFIG}" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=base_link \
    visual_odometry:=true \
    approx_sync:=true \
    approx_sync_max_interval:=0.2 \
    queue_size:=50 \
    qos:=2 \
    database_path:="${MAP_DIR}/rtabmap.db" &

sleep 5

echo "[+] Ожидание готовности RTAB-Map..."
for ((i=1; i<=SLAM_START_TIMEOUT; i++)); do
    if pgrep -f "rtabmap" >/dev/null || pgrep -f "rtabmap_ros" >/dev/null; then
        if [[ -f "${MAP_DIR}/rtabmap.db" ]]; then
            echo "[+] RTAB-Map готов (итерация $i)"
            break
        fi
    fi
    sleep "$TOPIC_CHECK_INTERVAL"
    if (( i == SLAM_START_TIMEOUT )); then
        echo "[-] RTAB-Map не запустился или не создал rtabmap.db"
        cleanup
        exit 1
    fi
done

# ---------- 4. Проверка одометрии ----------
echo "[+] Проверка наличия одометрии RTAB-Map..."
# В ROS2 rtabmap_ros может публиковать /odom или /rtabmap/odom
if ros2 topic list | grep -q "/rtabmap/odom"; then
    ODOM_TOPIC="/rtabmap/odom"
elif ros2 topic list | grep -q "/odom"; then
    ODOM_TOPIC="/odom"
else
    echo "[-] Топики одометрии (/rtabmap/odom или /odom) не найдены!"
    echo "[-] RTAB-Map будет игнорировать кадры без одометрии."
    cleanup
    exit 1
fi

echo "[+] Одометрия найдена: ${ODOM_TOPIC}"

# Лёгкий тест: получить одно сообщение одометрии
echo "[+] Проверка публикации одометрии (одно сообщение)..."
if ! ros2 topic echo "${ODOM_TOPIC}" --once >/dev/null 2>&1; then
    echo "[-] Одометрия не публикуется (нет данных по ${ODOM_TOPIC})!"
    cleanup
    exit 1
fi
echo "[+] Одометрия публикуется OK"

# ---------- Финальная информация ----------
echo "[+] SLAM успешно запущен"
echo "[+] Карта сохраняется в: ${MAP_DIR}/rtabmap.db"

echo "[+] Активные SLAM-топики:"
ros2 topic list | grep -E "(camera|scan|rtabmap|tf)" | head -20 || echo "(нет активных топиков)"

echo "[+] Активные ROS2 процессы:"
pgrep -f "ros2\|realsense\|ydlidar\|rtabmap" | wc -l

echo
echo "[+] Система готова к SLAM-картированию!"
echo "[+] Управляйте роботом для построения карты."
echo "[+] Для остановки и сохранения карты: CTRL+C"
echo "[+] Логи: $LOG_FILE"
echo "[+] Просмотр карты: rtabmap-databaseViewer ${MAP_DIR}/rtabmap.db"

wait