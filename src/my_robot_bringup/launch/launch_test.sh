#!/bin/bash
set -euo pipefail

USERNAME="${SUDO_USER:-$(whoami)}"
MAP_DIR="/home/${USERNAME}/maps/$(date +%Y-%m-%d_%H-%M-%S)"
LOG_FILE="${MAP_DIR}/slam.log"

SENSOR_START_TIMEOUT=30
TOPIC_CHECK_INTERVAL=1

mkdir -p "$MAP_DIR"
exec > >(tee -a "$LOG_FILE") 2>&1

echo "[+] Логирование: $LOG_FILE"
echo "[+] Время запуска: $(date)"
echo "[+] Карта: $MAP_DIR"

cleanup() {
    echo "[+] Остановка ROS2 процессов..."
    pkill -f realsense2_camera || true
    pkill -f ydlidar_ros2_driver || true
    pkill -f rgbd_odometry || true
    pkill -f rtabmap_ros || true
    pkill -f rtabmap || true
    sleep 1
    pkill -9 -f ros2 || true
    echo "[+] Все процессы остановлены"

    if [[ -f "${MAP_DIR}/rtabmap.db" ]]; then
        echo "[+] Размер карты: $(du -h "${MAP_DIR}/rtabmap.db" | cut -f1)"
    fi
}
trap cleanup INT TERM

# ---------- ROS2 ----------
if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck source=/dev/null
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
else
    echo "[-] ROS2 Jazzy не найден"
    exit 1
fi

# ---------- SENSORS ----------
echo "[+] Запуск сенсоров (RealSense + YDLIDAR)..."
ros2 launch my_robot_bringup bringup_sensors.launch.py &

sleep 5

echo "[+] Ожидание RGB-топика..."
for ((i=1;i<=SENSOR_START_TIMEOUT;i++)); do
    if ros2 topic list | grep -q "/camera/camera/color/image_raw"; then
        echo "[+] RGB OK"
        break
    fi
    sleep "$TOPIC_CHECK_INTERVAL"
done

echo "[+] Ожидание DEPTH-топика..."
for ((i=1;i<=SENSOR_START_TIMEOUT;i++)); do
    if ros2 topic list | grep -q "/camera/camera/depth/image_rect_raw"; then
        echo "[+] DEPTH OK"
        break
    fi
    sleep "$TOPIC_CHECK_INTERVAL"
done

echo "[+] Ожидание лидара..."
for ((i=1;i<=SENSOR_START_TIMEOUT;i++)); do
    if ros2 topic list | grep -q "^/scan$"; then
        echo "[+] /scan OK"
        break
    fi
    sleep "$TOPIC_CHECK_INTERVAL"
done

# ---------- TF ----------
echo "[+] Публикация статических TF..."
# Камера: base_link -> camera_link (дальше RealSense сам ведёт до color/depth frames)
ros2 run tf2_ros static_transform_publisher \
    0.075 0 0.02 0 0 0 base_link camera_link &

# Лидар: base_link -> laser_frame
ros2 run tf2_ros static_transform_publisher \
    0 0 0.05 0 0 0 base_link laser_frame &

sleep 5

# ---------- RGBD ODOM ----------
echo "[+] Запуск RGBD одометрии..."
ros2 run rtabmap_ros rgbd_odometry \
    --ros-args \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/depth/image_rect_raw \
    -r depth/camera_info:=/camera/camera/depth/camera_info \
    -p frame_id:=base_link \
    -p publish_tf:=true \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=0.8 &

sleep 3

# ---------- RTAB-Map ----------
echo "[+] Запуск RTAB-Map..."
ros2 run rtabmap_ros rtabmap \
    --ros-args \
    -r rgb/image:=/camera/camera/color/image_raw \
    -r rgb/camera_info:=/camera/camera/color/camera_info \
    -r depth/image:=/camera/camera/depth/image_rect_raw \
    -r depth/camera_info:=/camera/camera/depth/camera_info \
    -r scan:=/scan \
    -p frame_id:=base_link \
    -p database_path:="${MAP_DIR}/rtabmap.db" \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=0.8 \
    -p topic_queue_size:=50 \
    -p sync_queue_size:=50 \
    -p qos_image:=2 \
    -p qos_scan:=2 &

sleep 5

if [[ ! -f "${MAP_DIR}/rtabmap.db" ]]; then
    echo "[-] RTAB-Map пока не создал базу (нет хотя бы одного кадра)."
else
    echo "[+] RTAB-Map создал базу: ${MAP_DIR}/rtabmap.db"
fi

echo "[+] SLAM запущен. Води робота, смотри на лог."
echo "[+] Остановить и сохранить карту — CTRL+C"

wait