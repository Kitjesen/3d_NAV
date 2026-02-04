from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('remote_monitoring')
    config_file = os.path.join(pkg_dir, 'config', 'grpc_gateway.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'grpc_port',
            default_value='50051',
            description='gRPC server port'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),
        
        Node(
            package='remote_monitoring',
            executable='grpc_gateway',
            name='grpc_gateway',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
