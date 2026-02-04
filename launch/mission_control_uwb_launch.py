from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

"""Launch mission_control_uwb node for a single drone namespace.

This launch file assumes the perception + driver stack (tello_driver, MiDaS,
ArUco, etc.) is already running under the same namespace.

Arguments:
  drone_ns: (default: 'tello1') Namespace of the target drone.
  waypoints_file: (optional) Override waypoints JSON file path.
"""

def generate_launch_description():
    # Get package paths
    missions_dir = os.path.join(
        get_package_share_directory('tello_bringup'),
        'missions'
    )
    
    drone_ns_arg = DeclareLaunchArgument(
        'drone_ns', default_value='tello1',
        description='Namespace of the drone (e.g. tello1)')

    drone_ns = LaunchConfiguration('drone_ns')

    # Load mission configuration
    # Note: waypoints_file is configured per-drone in mission_waypoints_uwb.yaml
    # Relative filenames are resolved to tello_bringup/missions/ by the node
    mission_config = os.path.join(missions_dir, 'mission_waypoints_uwb.yaml')

    mission_control_node = Node(
        package='mission_control',
        executable='mission_control_uwb',
        name='mission_control_uwb',
        namespace=drone_ns,
        output='screen',
        parameters=[
            mission_config,
            {'drone_id': drone_ns}
        ]
    )

    return LaunchDescription([
        drone_ns_arg,
        mission_control_node
    ])
