import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # setup paths to the Nav2 bringup 
    pkg_description = get_package_share_directory('description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_startup = get_package_share_directory('startup')
    
    local_nav2_params_path = os.path.join(pkg_startup, 'launch', 'navigation_launch.py')
    
    nav2_params_path = os.path.join(pkg_description, 'src', 'startup', 'config', 'nav2_params.yaml')
    
    # 1. Locate and read the URDF file from the description package
    urdf_file_name = 'hershey.urdf'
    urdf_file_path = os.path.join(
        get_package_share_directory('description'),
        'urdf',
        urdf_file_name
    )

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Get the absolute path to your current working directory to easily find the config
    # (Assumes you run 'ros2 launch' from your ros2_ws root!)
    config_file_path = os.path.join(os.getcwd(), 'src', 'startup', 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        
        # 3. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 4. Arduino Bridge
        Node(
            package='mren203_arduino_bridge',
            executable='bridge',
            name='arduino_bridge',
            output='screen'
        ),
        
        # 5. SLLidar
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser', 
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # 6. Foxglove WebSocket Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765, 
                'address': '0.0.0.0' 
            }]
        ),

        # 7. SLAM Toolbox 
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link', 
                'scan_topic': '/scan',
                'transform_timeout': 0.5, 
                'mode': 'mapping',
                'map_update_interval': 2.0,
                'resolution': 0.05
            }]
        )

        # 8. Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_startup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params_path,
                'use_sim_time': 'False',
                'autostart': 'True'
            }.items()
        )
    ])