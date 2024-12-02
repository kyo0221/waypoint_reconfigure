from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('waypoint_reconfigure'),
        'config',
        'nav2_params_override.yaml'
    )

    param_change_node = Node(
        package = 'waypoint_reconfigure',
        namespace = 'waypoint_reconfigure_node',
        executable = 'waypoint_reconfigure',
        parameters = [config_file_path],
        output = 'screen'
    )

    return LaunchDescription([param_change_node])