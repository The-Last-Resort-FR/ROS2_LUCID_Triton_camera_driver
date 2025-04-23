import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        'config',
        'camera.yaml'
        )
        
    node=Node(
        package = 'camera_manager',
        name = 'camera_manager',
        executable = 'camera_manager',
        parameters = [config]
    )

    ld.add_action(node)
    
    return ld