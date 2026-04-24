from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'score_threshold',
            default_value='0.40',
            description='Umbral mínimo de confianza (0.0 - 1.0)'),

        DeclareLaunchArgument(
            'max_detections',
            default_value='10',
            description='Número máximo de detecciones por frame'),

        DeclareLaunchArgument(
            'publish_image',
            default_value='true',
            description='Publicar imagen anotada en /coral/image_detections'),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Topic de entrada de imágenes'),

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
