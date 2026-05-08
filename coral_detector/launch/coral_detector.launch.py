from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('score_threshold', default_value='0.40'),
        DeclareLaunchArgument('max_detections',  default_value='10'),
        DeclareLaunchArgument('publish_image',   default_value='true'),
        DeclareLaunchArgument('image_topic',     default_value='/camera/image_raw'),

        Node(
            package='coral_detector',
            executable='detector_node',
            name='coral_detector_node',
            output='screen',
            remappings=[
                ('/camera/image_raw', LaunchConfiguration('image_topic')),
            ],
            parameters=[{
                'score_threshold': LaunchConfiguration('score_threshold'),
                'max_detections':  LaunchConfiguration('max_detections'),
                'publish_image':   LaunchConfiguration('publish_image'),
            }],
        ),
    ])
