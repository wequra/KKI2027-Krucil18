"""Headless end-to-end smoke test: synthetic camera -> vision -> guidance -> bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_config = os.path.join(
        get_package_share_directory('perception'), 'config', 'vision_params.yaml')
    guidance_share = get_package_share_directory('guidance_control')
    guidance_config = os.path.join(guidance_share, 'config', 'guidance_params.yaml')
    rc_override_config = os.path.join(
        guidance_share, 'config', 'rc_override_params.yaml')

    total_gates_arg = DeclareLaunchArgument('total_gates', default_value='10')
    show_gui_arg = DeclareLaunchArgument('show_gui', default_value='false')
    control_backend_arg = DeclareLaunchArgument(
        'control_backend', default_value='velocity',
        description='Backend actuator: velocity atau rc_override.')

    synthetic_camera = Node(
        package='simulation', executable='synthetic_buoy_camera.py',
        name='synthetic_buoy_camera', output='screen',
        parameters=[{'total_gates': LaunchConfiguration('total_gates')}])
    detector = Node(
        package='perception', executable='vision_detector_node',
        name='vision_detector_node', output='screen',
        parameters=[vision_config],
        remappings=[('camera/image_raw', '/camera/image_raw'),
                    ('obstacles', '/asv/obstacles')])
    centering = Node(
        package='guidance_control', executable='buoy_centering_node',
        name='buoy_centering_node', output='screen',
        parameters=[guidance_config, {
            'total_gates': LaunchConfiguration('total_gates'),
            'show_gui': LaunchConfiguration('show_gui'),
            'use_object_tracking': False,  # synthetic HSV regression only
        }])
    bridge = Node(
        package='mavros_bridge', executable='mavros_bridge_node',
        name='mavros_bridge_node', output='screen',
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('control_backend'), 'velocity')),
        parameters=[{'configure_mavros_velocity_frame': False}])
    rc_override = Node(
        package='guidance_control', executable='cmd_vel_to_rc_override_node',
        name='cmd_vel_to_rc_override_node', output='screen',
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('control_backend'), 'rc_override')),
        parameters=[rc_override_config])

    return LaunchDescription([
        total_gates_arg, show_gui_arg, control_backend_arg,
        synthetic_camera, detector, centering, bridge, rc_override,
    ])
