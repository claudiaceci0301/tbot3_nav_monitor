from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('tbot3_nav_monitor'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package = 'tbot3_nav_monitor',
            executable = 'metric_collector',
            name = 'metric_collector_node',
            parameters = [config],
            output = 'screen'
    ),
    Node(
        package = 'tbot3_nav_monitor',
        executable = 'metric_collector',
        name = 'adaptive_controller_node',
        parameters = [config],
        output = 'screen'
    ),
    Node(
        package='tbot3_nav_monitor',
        executable='metric_collector',
        name='data_logger_node',
        parameters=[config],
        output='screen'
    )

])