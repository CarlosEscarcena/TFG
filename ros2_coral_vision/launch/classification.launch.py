"""
Launch file: clasificación de imágenes con Google Coral
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path',      default_value=''),
        DeclareLaunchArgument('labels_path',     default_value=''),
        DeclareLaunchArgument('input_topic',     default_value='/camera/image_raw'),
        DeclareLaunchArgument('score_threshold', default_value='0.3'),
        DeclareLaunchArgument('use_edgetpu',     default_value='true'),

        Node(
            package='ros2_coral_vision',
            executable='coral_vision_node',
            name='coral_vision_node',
            output='screen',
            parameters=[{
                'model_path':      LaunchConfiguration('model_path'),
                'labels_path':     LaunchConfiguration('labels_path'),
                'task':            'classification',
                'input_topic':     LaunchConfiguration('input_topic'),
                'output_topic':    '/coral/classification',
                'score_threshold': LaunchConfiguration('score_threshold'),
                'use_edgetpu':     LaunchConfiguration('use_edgetpu'),
                'publish_annotated': False,
            }],
        ),
    ])
