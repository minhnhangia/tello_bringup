from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# Launch nodes required for a single drone


def generate_launch_description():
    # Get the package directory
    pkg_tello_bringup = get_package_share_directory('tello_bringup')
    
    # Path to the parameter file
    params_file = os.path.join(pkg_tello_bringup, 'config', 'drone_params.yaml')
    
    # Drone namespace
    dr1_ns = 'drone1'

    return LaunchDescription([
        Node(
            package='tello_driver', 
            executable='tello_driver_main', 
            output='screen',
            name='tello_driver1', 
            namespace=dr1_ns, 
            parameters=[params_file]
        ),
    ])