from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(os.path.expanduser('~'), 'my_robot_ws', 'maps', 'map.yaml')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'params_file': nav2_params,
            'map': map_yaml,
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([nav2_bringup])