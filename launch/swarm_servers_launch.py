from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_tello_bringup = get_package_share_directory('tello_bringup')
    config_file = os.path.join(pkg_tello_bringup, 'config', 'drone_params.yaml')

    # Load the drones list
    with open(config_file, 'r') as f:
        drones_config = yaml.safe_load(f)['drones']

    nodes = []

    drone_ids = [drone['name'] for drone in drones_config]

    # Takeoff Server (single node for all drones)
    takeoff_server_params = {
		'drone_ids': drone_ids,
	}

    nodes.append(Node(
		package='tello_swarm',
		executable='takeoff_server',
		name='takeoff_server',
		output='screen',
		parameters=[takeoff_server_params],
        respawn=True
	))

    return LaunchDescription(nodes)