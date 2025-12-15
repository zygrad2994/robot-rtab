from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(os.path.expanduser('~'), 'my_robot_ws', 'maps', 'map.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}]
    )

    nav2_bringup = Node(
        package='nav2_bringup',
        executable='bringup_launch',
        name='nav2_bringup',
        output='screen',
        parameters=[nav2_params]
    )

    ld = LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(nav2_bringup)
    return ld
