"""Complete ASV mission orchestration around the unchanged buoy algorithm."""
import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gshare = get_package_share_directory('guidance_control')
    bshare = get_package_share_directory('bringup')
    guidance = os.path.join(gshare, 'config', 'guidance_params.yaml')
    rc = os.path.join(gshare, 'config', 'rc_override_params.yaml')
    mission = os.path.join(bshare, 'config', 'full_mission_params.yaml')
    args = [
        DeclareLaunchArgument('use_mavros', default_value='true'),
        DeclareLaunchArgument('main_camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('underwater_camera_device', default_value='/dev/video1'),
        DeclareLaunchArgument('control_backend', default_value='rc_override'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:115200'),
        DeclareLaunchArgument('stream_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('stream_port', default_value='8080'),
        DeclareLaunchArgument('gcs_bind_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('gcs_port', default_value='8090'),
        DeclareLaunchArgument('show_gui', default_value='false'),
        DeclareLaunchArgument('course_layout', default_value='A'),
        DeclareLaunchArgument('enable_gcs_media_bridge', default_value='true'),
        DeclareLaunchArgument('gcs_media_bridge_script', default_value='/home/rie/dagozilla-gcs/jetson/gcs_media_bridge_node.py'),
        DeclareLaunchArgument('gcs_media_bridge_params', default_value='/home/rie/dagozilla-gcs/jetson/bridge_params.yaml'),
        DeclareLaunchArgument('gcs_api_url', default_value='http://<GCS_IP>:8000'),
        DeclareLaunchArgument('jetson_api_key', default_value='test_key'),
        DeclareLaunchArgument('surface_stream_rtsp_url', default_value='rtsp://<GCS_IP>:8554/surface'),
    ]
    nodes = [
        Node(package='perception', executable='opencv_camera_node', name='main_camera', output='screen', parameters=[{'video_device': LaunchConfiguration('main_camera_device'), 'frame_id': 'main_camera'}], remappings=[('camera/image_raw', '/camera/image_raw')]),
        Node(package='perception', executable='opencv_camera_node', name='underwater_camera', output='screen', parameters=[{'video_device': LaunchConfiguration('underwater_camera_device'), 'frame_id': 'underwater_camera'}], remappings=[('camera/image_raw', '/underwater_camera/image_raw')]),
        Node(package='perception', executable='ball_detector_node.py', name='vision_detector_node', output='screen', remappings=[('camera/image_raw', '/camera/image_raw'), ('obstacles', '/asv/obstacles')]),
        Node(package='guidance_control', executable='buoy_centering_node', name='buoy_centering_node', output='screen', parameters=[guidance, {'show_gui': ParameterValue(LaunchConfiguration('show_gui'), value_type=bool), 'course_layout': LaunchConfiguration('course_layout')}], remappings=[('/asv/cmd_vel', '/asv/cmd_vel/buoy')]),
        Node(package='mission', executable='mission_manager_node', name='mission_manager_node', output='screen', parameters=[mission]),
        Node(package='mission', executable='mission_cmd_mux_node.py', name='mission_cmd_mux_node', output='screen', parameters=[mission]),
        Node(package='mission', executable='mission_controller_node.py', name='underwater_imaging_controller', output='screen', parameters=[mission, {'controller_mode': 'underwater'}]),
        Node(package='mission', executable='mission_controller_node.py', name='surface_imaging_controller', output='screen', parameters=[mission, {'controller_mode': 'surface'}]),
        Node(package='mission', executable='mission_controller_node.py', name='docking_controller', output='screen', parameters=[mission, {'controller_mode': 'docking'}]),
        Node(package='guidance_control', executable='cmd_vel_to_rc_override_node', name='cmd_vel_to_rc_override_node', output='screen', condition=IfCondition(EqualsSubstitution(LaunchConfiguration('control_backend'), 'rc_override')), parameters=[rc]),
        Node(package='mavros_bridge', executable='mavros_bridge_node', name='mavros_bridge_node', output='screen', condition=IfCondition(EqualsSubstitution(LaunchConfiguration('control_backend'), 'velocity')), parameters=[{'configure_mavros_velocity_frame': ParameterValue(LaunchConfiguration('use_mavros'), value_type=bool)}]),
        Node(package='perception', executable='camera_rtsp_streamer_node.py', name='camera_rtsp_streamer', output='screen', parameters=[{'image_topic': '/camera/image_raw', 'host': LaunchConfiguration('stream_host'), 'port': ParameterValue(LaunchConfiguration('stream_port'), value_type=int)}]),
        Node(package='mission', executable='gcs_bridge_node.py', name='gcs_bridge_node', output='screen', parameters=[mission, {'bind_host': LaunchConfiguration('gcs_bind_host'), 'port': ParameterValue(LaunchConfiguration('gcs_port'), value_type=int)}]),
        ExecuteProcess(cmd=['python3', LaunchConfiguration('gcs_media_bridge_script'), '--ros-args', '--params-file', LaunchConfiguration('gcs_media_bridge_params'), '-p', ['gcs_api_url:=', LaunchConfiguration('gcs_api_url')], '-p', ['jetson_api_key:=', LaunchConfiguration('jetson_api_key')], '-p', ['surface_stream_rtsp_url:=', LaunchConfiguration('surface_stream_rtsp_url')]], output='screen', condition=IfCondition(LaunchConfiguration('enable_gcs_media_bridge')), name='gcs_media_bridge'),
    ]
    try:
        mavros = get_package_share_directory('mavros')
        nodes.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(mavros, 'launch', 'apm.launch')), condition=IfCondition(LaunchConfiguration('use_mavros')), launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()))
    except PackageNotFoundError:
        pass
    return LaunchDescription(args + nodes)
