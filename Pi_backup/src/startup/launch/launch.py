from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # 1. Arduino Bridge
        Node(
            package='mren203_arduino_bridge',
            executable='bridge',
            name='arduino_bridge',
            output='screen'
        ),
        
        # 2. SLLidar (Slamtec Lidar)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # 3. Foxglove WebSocket Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765, 
                'address': '0.0.0.0' 
            }]
        )
    ])