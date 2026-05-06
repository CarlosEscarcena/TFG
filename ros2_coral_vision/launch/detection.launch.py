"""
Launch file: detección de objetos con Google Coral
Uso:
  ros2 launch ros2_coral_vision detection.launch.py \
    model_path:=/ruta/modelo_edgetpu.tflite \
    labels_path:=/ruta/labels.txt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Argumentos ────────────────────────────────────────────────────────
        DeclareLaunchArgument('model_path',   default_value='',
                              description='Ruta al archivo .tflite (Edge TPU)'),
        DeclareLaunchArgument('labels_path',  default_value='',
                              description='Ruta al archivo de etiquetas (.txt)'),
        DeclareLaunchArgument('input_topic',  default_value='/camera/image_raw',
                              description='Topic de entrada de imágenes'),
        DeclareLaunchArgument('output_topic',  default_value='/coral/image_detections',
                              description='Topic de entrada de imágenes'),
        DeclareLaunchArgument('score_threshold', default_value='0.5',
                              description='Umbral de confianza (0.0 - 1.0)'),
        DeclareLaunchArgument('use_edgetpu',  default_value='true',
                              description='Usar Edge TPU (false = CPU)'),
        DeclareLaunchArgument('publish_annotated', default_value='true',
                              description='Publicar imagen con anotaciones'),

        # ── Nodo ──────────────────────────────────────────────────────────────
        Node(
            package='ros2_coral_vision',
            executable='coral_vision_node',
            name='coral_vision_node',
            output='screen',
            parameters=[{
                'model_path':        LaunchConfiguration('model_path'),
                'labels_path':       LaunchConfiguration('labels_path'),
                'task':              'detection',
                'input_topic':       LaunchConfiguration('input_topic'),
                'output_topic':      LaunchConfiguration('output_topic'),
                'score_threshold':   LaunchConfiguration('score_threshold'),
                'use_edgetpu':       LaunchConfiguration('use_edgetpu'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }],
        ),
    ])
