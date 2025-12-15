from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'bringup_sensors.launch.py'))
    )

    rtabmap_params = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')

    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        arguments=['--delete_db_on_start']
    )

    rtabmapviz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(bringup_launch)
    ld.add_action(rtabmap_node)
    ld.add_action(rtabmapviz)
    return ld
