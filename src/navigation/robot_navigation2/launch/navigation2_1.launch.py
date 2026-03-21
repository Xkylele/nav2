import os
import launch
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource

def generate_launch_description():
    fishbot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='False')
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params_1.yaml'))
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'map.yaml'))

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),
        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file to load'
        ),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,                  
            }.items(),
        ),

        Node(
                package="map_trans_pkg",
                executable="map_publisher_node",
                name="map_for_nav2",
                output="screen",
                parameters=[
                    {
                        "map_path": map_yaml_path,      # Same YAML used by Nav2
                        "publish_topic": "/map",
                        "frame_id": "map",
                        "publish_interval": 10.0,
                    }
                ],
            ),
        # /////////////添加terrain相关/////////////
        launch.actions.IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
        )
        ),
        # launch.actions.IncludeLaunchDescription(
        #     FrontendLaunchDescriptionSource(os.path.join(
        #     get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
        #     ),
        #     # launch_arguments={
        #     # 'checkTerrainConn': checkTerrainConn,
        #     # }.items()
        # ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
