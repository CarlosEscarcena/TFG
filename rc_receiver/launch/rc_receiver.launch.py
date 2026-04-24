from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('rc_receiver'),
        'config',
        'rc_receiver_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('throttle_pin', default_value='13'),
        DeclareLaunchArgument('steering_pin', default_value='17'),

        Node(
            package='rc_receiver',
            executable='rc_receiver_node',
            name='rc_receiver_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                config,
                {
                    'throttle_pin': LaunchConfiguration('throttle_pin'),
                    'steering_pin': LaunchConfiguration('steering_pin'),
                }
            ],
        ),
    ])