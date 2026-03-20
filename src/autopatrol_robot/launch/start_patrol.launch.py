import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    auto_patrol_share = get_package_share_directory("autopatrol_robot")

    patrol_config_path = os.path.join(auto_patrol_share,"config","patrol_config.yaml")

    return LaunchDescription([
        Node(
            package='autopatrol_robot',
            executable='patrol_node',
            parameters=[patrol_config_path]
        ),
    ])
