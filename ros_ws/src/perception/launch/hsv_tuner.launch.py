"""Launch file untuk HSV Threshold Calibration Tool (perception).

Contoh penggunaan:
    ros2 launch perception hsv_tuner.launch.py
    ros2 launch perception hsv_tuner.launch.py camera_device:=/dev/video0
    ros2 launch perception hsv_tuner.launch.py use_camera:=false image_topic:=/camera/image_raw
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Topic sensor_msgs/Image dari kamera.')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Jalankan driver opencv_camera_node untuk USB webcam.')
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Device kamera V4L2 (misal: /dev/video0, 0, 1).')

    camera_node = Node(
        package='perception',
        executable='opencv_camera_node',
        name='opencv_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_camera')),
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'width': 640,
            'height': 480,
        }],
        remappings=[
            ('camera/image_raw', LaunchConfiguration('image_topic')),
        ]
    )

    tuner_node = Node(
        package='perception',
        executable='hsv_tuner_node',
        name='hsv_tuner_node',
        output='screen',
        parameters=[{'show_gui': True}],
        remappings=[
            ('camera/image_raw', LaunchConfiguration('image_topic')),
        ]
    )

    return LaunchDescription([
        image_topic_arg,
        use_camera_arg,
        camera_device_arg,
        camera_node,
        tuner_node,
    ])
