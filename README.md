Проект: RTAB-Map + ROS2 Humble на Ubuntu 22.04
---------------------------------------------

1) Предварительные требования
   - Ubuntu 22.04 (на Raspberry Pi 5 или ПК)
   - Доступ к интернету для установки пакетов
   - Сенсоры: Intel RealSense D435i, YDLidar X3 (или аналог), опционально Dahua RTSP

2) Установка (автоматически)
   - Скопировать install_my_robot.sh в ~/my_robot_ws/
   - Выполнить:
       chmod +x ~/my_robot_ws/install_my_robot.sh
       ~/my_robot_ws/install_my_robot.sh
   - Открыть новый терминал или выполнить: source ~/.bashrc

3) Сборка и подготовка
   - Если добавляли/редактировали файлы в src, выполнить:
       cd ~/my_robot_ws
       colcon build --symlink-install
       source install/setup.bash

4) Запуск сенсоров
   - Запустить:
       ros2 launch my_robot_bringup bringup_sensors.launch.py
   - Проверить топики:
       ros2 topic list
     Ожидаемые: /camera/color/image_raw, /camera/depth/image_rect_raw, /camera/imu, /scan

5) Mapping (RTAB-Map)
   - Запустить:
       ros2 launch my_robot_bringup rtabmap_mapping.launch.py
   - Открыть rviz2 и добавить PointCloud2, TF, Camera, Map
   - Вести робота вручную по окружению, пока карта не будет достаточной
пункты 5 и 6 опираться на скрипт 
6) Сохранение карты
   - Сохранить базу RTAB-Map:
       ros2 service call /rtabmap/save_map std_srvs/srv/Empty "{}"
   - Скопировать .db в ~/my_robot_ws/maps/my_map.db

7) Экспорт occupancy grid (map.png + map.yaml)
   - На машине с GUI:
       rtabmap-databaseViewer ~/my_robot_ws/maps/my_map.db
     В GUI: File → Export → Export occupancy grid → сохранить map.png и map.yaml в ~/my_robot_ws/maps/

8) Навигация (Nav2) В РАЗРАБОТКЕ! МЕТОД НЕ РАБОЧИЙ!
   - Запустить:
       ros2 launch my_robot_bringup nav2_bringup.launch.py
   - В rviz2: задать 2D Pose Estimate (инициализация локализации), затем 2D Nav Goal
   - Робот будет планировать и двигаться по карте без GPS

9) Советы по отладке
   - Проверяйте TF (ros2 run tf2_tools view_frames)
   - Если Pi перегружен — уменьшайте Kp/MaxFeatures и увеличивайте RGBD/LinearUpdate
   - Для тяжёлых карт — переносите mapping на более мощный ПК и публикуйте сенсоры по сети (ROS2 DDS)

10) Файлы конфигурации
   - ~/my_robot_ws/src/my_robot_bringup/config/rtabmap_params.yaml
   - ~/my_robot_ws/src/my_robot_bringup/config/nav2_params.yaml

