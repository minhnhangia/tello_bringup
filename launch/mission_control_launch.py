from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""Launch mission_control node for a single drone namespace.

This launch file assumes the perception + driver stack (tello_driver, MiDaS,
ArUco, etc.) is already running under the same namespace.

Arguments:
  drone_ns: (default: 'tello1') Namespace of the target drone.
"""

def generate_launch_description():
    drone_ns_arg = DeclareLaunchArgument(
        'drone_ns', default_value='tello1',
        description='Namespace of the drone (e.g. tello1)')

    drone_ns = LaunchConfiguration('drone_ns')

    mission_control_node = Node(
        package='mission_control',
        executable='mission_control',
        name='mission_control',
        namespace=drone_ns,
        output='screen',
        parameters=[{
            'drone_id': drone_ns,
            # Expose key tunables here (optional overrides). If omitted the node uses its defaults.
            # 'yaw_speed': 0.5,
            # 'forward_speed': 0.2,
            # 'sideway_speed': 0.1,
        }]
    )

    return LaunchDescription([
        drone_ns_arg,
        mission_control_node
    ])
