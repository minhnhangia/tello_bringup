#!/usr/bin/env python3
"""
Launch file for UWB position republisher node.
Reads drone configuration from drone_params_uwb.yaml and republishes
LinkTrack UWB position data to individual drone topics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # Get package directories
    pkg_tello_bringup = get_package_share_directory('tello_bringup')
    config_file = os.path.join(pkg_tello_bringup, 'config', 'drone_params_uwb.yaml')

    # Load the drones configuration
    with open(config_file, 'r') as f:
        drones_config = yaml.safe_load(f)['drones']

    # Extract drone names and UWB tag IDs from the configuration
    drone_names = []
    uwb_tag_ids = []
    
    for drone in drones_config:
        drone_names.append(drone['name'])
        uwb_tag_ids.append(drone['uwb_tag_id'])

    # Create the UWB republisher node
    uwb_republisher_node = Node(
        package='tello_uwb',
        executable='uwb_republisher',
        name='uwb_republisher',
        output='screen',
        parameters=[{
            'frame_id': 'world',
            'expected_role': 2,
            'input_topic': '/nlink_linktrack_anchorframe0',
            'drone_names': drone_names,
            'uwb_tag_ids': uwb_tag_ids,
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        uwb_republisher_node,
    ])
