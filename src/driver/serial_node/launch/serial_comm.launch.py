from launch import LaunchDescription
from launch_ros.actions import Node
# 和物理插的位置有关 记得插离开关近的
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_node',
            executable='serial_twist_publisher',
            name='serial_twist_publisher',
            parameters=[
                {'port': '/dev/usb_serial_0'},
                {'baudrate': 115200},
                {'linear_scale': 1000.0},     
                {'angular_scale': 1000.0},   
            ],
            output='screen'
        )
    ])