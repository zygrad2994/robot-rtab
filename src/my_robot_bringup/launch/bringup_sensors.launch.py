from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # RealSense
    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_pkg, 'launch', 'rs_launch.py'))
    )
    ld.add_action(realsense_launch)

    # YDLidar (замени имя пакета при необходимости)
    try:
        ydlidar_pkg = get_package_share_directory('ydlidar_ros2_driver')
        ydlidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ydlidar_pkg, 'launch', 'ydlidar.launch.py'))
        )
        ld.add_action(ydlidar_launch)
    except Exception:
        pass

    return ld
