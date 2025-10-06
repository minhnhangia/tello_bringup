from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
	# Path to drone params yaml
	pkg_tello_bringup = get_package_share_directory('tello_bringup')
	config_file = os.path.join(pkg_tello_bringup, 'config', 'drone_params.yaml')

	# Load drone names
	with open(config_file, 'r') as f:
		drones_config = yaml.safe_load(f)['drones']
	drone_ids = [drone['name'] for drone in drones_config]

	# multi_midas_node parameters
	params = {
		'drone_ids': drone_ids,
		# 'model_type': 'MiDaS_small'
	}

	node = Node(
		package='tello_midas',
		executable='multi_midas_inference',
		name='multi_midas_inference',
		output='screen',
		parameters=[params]
	)

	return LaunchDescription([node])
