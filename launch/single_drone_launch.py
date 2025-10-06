from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_tello_bringup = get_package_share_directory('tello_bringup')
    config_file = os.path.join(pkg_tello_bringup, 'config', 'drone_params.yaml')

    # camera info file (shared for all drones here)
    pkg_tello_driver = get_package_share_directory('tello_driver')
    camera_info_path = os.path.join(pkg_tello_driver, 'cfg', 'camera_info.yaml')
    camera_info_path_down = os.path.join(pkg_tello_driver, 'cfg', 'camera_down_info.yaml')

    # Load the drones list
    with open(config_file, 'r') as f:
        drones_config = yaml.safe_load(f)['drones']

    nodes = []
    for drone in drones_config[0:1]:
        ns = drone['name']
        node_name = drone['node_name']

        # Build the parameter dict for this drone
        params = {
            'drone_ip': drone['drone_ip'],
            'command_port': drone['command_port'],
            'drone_port': drone['drone_port'],
            'data_port': drone['data_port'],
            'video_port': drone['video_port'],
            'camera_info_path': camera_info_path,
            'camera_info_path_down' : camera_info_path_down
            'odom_frame_id': drone['odom_frame_id'],
            'base_frame_id': drone['base_frame_id'],
            'use_sim_time': False
        }

        nodes.append(Node(
            package='tello_driver',
            executable='tello_driver_main',
            name=node_name,
            namespace=ns,
            output='screen',
            parameters=[params]
        ))

    return LaunchDescription(nodes)