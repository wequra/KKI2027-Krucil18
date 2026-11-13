"""
Launch file utama: menjalankan MAVROS (koneksi ke Pixhawk) + seluruh node ASV.
Jalankan dengan:
    ros2 launch bringup bringup.launch.py fcu_url:=/dev/ttyACM0:115200
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:115200',
        description='Port serial/UDP koneksi ke Pixhawk')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Topic sensor_msgs/Image untuk detector buoy.')

    mission_config = os.path.join(
        get_package_share_directory('mission'),
        'config',
        'mission_params.yaml')

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mavros'),
                         'launch', 'apm.launch')
        ),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()
    )

    nodes = [
        Node(package='mavros_bridge', executable='mavros_bridge_node', output='screen'),
        Node(package='guidance_control', executable='waypoint_follower_node', output='screen'),
        Node(
            package='mission',
            executable='mission_manager_node',
            output='screen',
            parameters=[mission_config]),
        Node(
            package='perception',
            executable='ball_detector_node.py',
            output='screen',
            remappings=[
                ('camera/image_raw', LaunchConfiguration('image_topic')),
                ('obstacles', '/asv/obstacles'),
            ]),
    ]

    return LaunchDescription([fcu_url_arg, image_topic_arg, mavros_launch] + nodes)
