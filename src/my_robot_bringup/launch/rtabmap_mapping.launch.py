from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Namespace камеры RealSense'
    )

    camera_name = LaunchConfiguration('camera_name')

    realsense_pkg = get_package_share_directory('realsense2_camera')
    rs_launch_path = os.path.join(realsense_pkg, 'launch', 'rs_launch.py')

    realsense_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={
                'camera_name': camera_name,

                # ВАЖНО: выравниваем глубину по цвету
                'align_depth.enable': 'true',

                # Режем всё лишнее
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'pointcloud.enable': 'false',

                # Профиль под твой FPS и ресурсы
                'rgb_camera.color_profile': '640x360x15',
                'depth_module.depth_profile': '640x360x15',

                # Синхронизированный стрим
                'enable_sync': 'true',

                # Сброс девайса при старте
                'initial_reset': 'true',
            }.items()
        )
    ])

    return LaunchDescription([
        camera_name_arg,
        realsense_group
    ])