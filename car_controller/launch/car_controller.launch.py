from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('car_controller'),
        'config',
        'car_params.yaml'
    )

    return LaunchDescription([
        # Argumentos sobreescribibles desde línea de comandos
        DeclareLaunchArgument('steering_pin',      default_value='12'),
        DeclareLaunchArgument('throttle_pin',      default_value='19'),
        DeclareLaunchArgument('cmd_vel_timeout',   default_value='0.5'),

        Node(
            package='car_controller',
            executable='car_controller_node',
            name='car_controller_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                config,
                {
                    'steering_pin':    LaunchConfiguration('steering_pin'),
                    'throttle_pin':    LaunchConfiguration('throttle_pin'),
                    'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
                }
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),  # cambiar aquí si tu topic es diferente
            ],
        ),
    ])