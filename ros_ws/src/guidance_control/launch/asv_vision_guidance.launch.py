"""Unified Launch File untuk ASV Vision Guidance KKI 2026.

Menjalankan:
1. Driver USB Camera V4L2 (opsional, use_camera:=true)
2. YOLO + ByteTrack buoy detector (perception)
3. Buoy Centering Guidance Node (guidance_control)
4. MAVROS Bridge Node (mavros_bridge)
5. MAVROS APM Driver (opsional, use_mavros:=true)
6. HSV Tuner GUI Tool (opsional, run_hsv_tuner:=true)

Contoh penggunaan:
  # 1. Tes kamera lokal + HSV thresholding + centering output (tanpa Pixhawk):
  ros2 launch guidance_control asv_vision_guidance.launch.py camera_device:=/dev/video0

  # 2. Tes dengan tuning HSV GUI aktif:
  ros2 launch guidance_control asv_vision_guidance.launch.py run_hsv_tuner:=true

  # 3. Mode komplit di kapal asli dengan MAVROS Pixhawk terhubung:
  ros2 launch guidance_control asv_vision_guidance.launch.py use_mavros:=true fcu_url:=/dev/ttyACM0:115200
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    perception_share = get_package_share_directory('perception')
    guidance_share = get_package_share_directory('guidance_control')

    guidance_config = os.path.join(guidance_share, 'config', 'guidance_params.yaml')
    rc_override_config = os.path.join(
        guidance_share, 'config', 'rc_override_params.yaml')

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='Port V4L2 USB camera (misal: /dev/video0, /dev/video1)')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera', default_value='true',
        description='Jalankan driver v4l2_camera untuk USB webcam.')
    use_mavros_arg = DeclareLaunchArgument(
        'use_mavros', default_value='false',
        description='Set true jika Pixhawk fisik terhubung via MAVROS.')
    control_backend_arg = DeclareLaunchArgument(
        'control_backend', default_value='velocity',
        description='Backend actuator: velocity atau rc_override.')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:115200',
        description='Port serial/UDP koneksi Pixhawk MAVROS.')
    run_hsv_tuner_arg = DeclareLaunchArgument(
        'run_hsv_tuner', default_value='false',
        description='Set true untuk membuka GUI slider HSV Tuner.')
    show_gui_arg = DeclareLaunchArgument(
        'show_gui', default_value='false',
        description='Tampilkan OpenCV GUI. Default false agar aman di Docker/headless.')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Topic sensor_msgs/Image dari kamera.')
    obstacle_topic_arg = DeclareLaunchArgument(
        'obstacle_topic', default_value='/asv/obstacles',
        description='Topic ObstacleArray dari vision detector.')
    run_streamer_arg = DeclareLaunchArgument(
        'run_streamer', default_value='true',
        description='Jalankan HTTP MJPEG streamer untuk web GCS.')
    stream_host_arg = DeclareLaunchArgument(
        'stream_host', default_value='0.0.0.0',
        description='Alamat bind HTTP MJPEG streamer.')
    stream_port_arg = DeclareLaunchArgument(
        'stream_port', default_value='8080',
        description='Port HTTP MJPEG streamer.')

    # 1. Driver kamera USB OpenCV (perception)
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

    # 2. Vision Detector (perception)
    detector_node = Node(
        package='perception',
        executable='ball_detector_node.py',
        name='vision_detector_node',
        output='screen',
        parameters=[os.path.join(perception_share, 'config', 'tracking_params.yaml')],
        remappings=[
            ('camera/image_raw', LaunchConfiguration('image_topic')),
            ('obstacles', LaunchConfiguration('obstacle_topic')),
        ]
    )

    # 3. Buoy Centering Node (guidance_control)
    centering_node = Node(
        package='guidance_control',
        executable='buoy_centering_node',
        name='buoy_centering_node',
        output='screen',
        parameters=[guidance_config, {
            'show_gui': LaunchConfiguration('show_gui'),
        }],
        remappings=[
            ('/asv/obstacles', LaunchConfiguration('obstacle_topic')),
            ('/camera/image_raw', LaunchConfiguration('image_topic')),
        ]
    )

    # 4. MAVROS Bridge Node
    bridge_node = Node(
        package='mavros_bridge',
        executable='mavros_bridge_node',
        name='mavros_bridge_node',
        output='screen',
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('control_backend'), 'velocity')),
        parameters=[{
            'configure_mavros_velocity_frame': ParameterValue(
                LaunchConfiguration('use_mavros'), value_type=bool),
            'mavros_velocity_frame': 'BODY_NED',
        }]
    )

    rc_override_node = Node(
        package='guidance_control',
        executable='cmd_vel_to_rc_override_node',
        name='cmd_vel_to_rc_override_node',
        output='screen',
        condition=IfCondition(EqualsSubstitution(
            LaunchConfiguration('control_backend'), 'rc_override')),
        parameters=[rc_override_config]
    )

    # 5. Interactive HSV Tuner Node (opsional)
    hsv_tuner_node = Node(
        package='perception',
        executable='hsv_tuner_node',
        name='hsv_tuner_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_hsv_tuner')),
        remappings=[
            ('camera/image_raw', LaunchConfiguration('image_topic')),
        ]
    )

    # 7. HTTP MJPEG output for the web GCS. Browsers cannot play RTSP directly.
    streamer_node = Node(
        package='perception',
        executable='camera_rtsp_streamer_node.py',
        name='camera_rtsp_streamer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_streamer')),
        parameters=[{
            'image_topic': '/asv/guidance_debug',
            'host': LaunchConfiguration('stream_host'),
            'port': ParameterValue(LaunchConfiguration('stream_port'), value_type=int),
        }]
    )

    # 6. MAVROS APM driver (opsional bila use_mavros:=true)
    launch_entities = [
        camera_device_arg,
        use_camera_arg,
        use_mavros_arg,
        control_backend_arg,
        fcu_url_arg,
        run_hsv_tuner_arg,
        show_gui_arg,
        image_topic_arg,
        obstacle_topic_arg,
        run_streamer_arg,
        stream_host_arg,
        stream_port_arg,
        camera_node,
        detector_node,
        centering_node,
        bridge_node,
        rc_override_node,
        hsv_tuner_node,
        streamer_node,
    ]

    try:
        mavros_share = get_package_share_directory('mavros')
        mavros_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_share, 'launch', 'apm.launch')
            ),
            condition=IfCondition(LaunchConfiguration('use_mavros')),
            launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()
        )
        launch_entities.append(mavros_launch)
    except PackageNotFoundError:
        pass

    return LaunchDescription(launch_entities)
