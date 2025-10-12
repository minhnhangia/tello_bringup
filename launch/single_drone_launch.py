from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

"""Single drone bringup including:
 - tello_driver (already parameterized via drone_params.yaml)
 - MiDaS depth inference (tello_midas/midas_inference)
 - MiDaS depth analysis (tello_midas/midas_analysis)
 - ArUco tracker (aruco_opencv/aruco_tracker_autostart)

All auxiliary nodes are started inside the drone namespace (e.g. 'tello1')
"""

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
            'camera_info_path_down' : camera_info_path_down,
            'odom_frame_id': drone['odom_frame_id'],
            'base_frame_id': drone['base_frame_id'],
            'is_ext_tof_attached': drone['is_ext_tof_attached'],
            'use_sim_time': False
        }

        # 1. Tello driver
        nodes.append(Node(
            package='tello_driver',
            executable='tello_driver_main',
            name=node_name,
            namespace=ns,
            output='screen',
            parameters=[params]
        ))

        # 2. MiDaS inference (depth)
        nodes.append(Node(
            package='tello_midas',
            executable='midas_inference',
            name='midas_inference',
            namespace=ns,
            output='screen',
            parameters=[{
                'model_type': 'MiDaS_small',
                'input_topic': 'image_raw',
                'output_raw_topic': 'depth/raw'
            }],
            respawn=True
        ))

        # 3. MiDaS analysis
        nodes.append(Node(
            package='tello_midas',
            executable='midas_analysis',
            name='midas_analysis',
            namespace=ns,
            output='screen',
            parameters=[{
                'input_depth_topic': 'depth/raw',
                'output_colormap_topic': 'depth/colormap',
                'output_annotated_colormap_topic': 'depth/colormap_annotated',
                'output_colormap_analysis_topic': 'depth/analysis'
            }],
            respawn=True
        ))

        # 4. ArUco tracker
        #    Load existing parameter file, override cam_base_topic to relative so namespace applies.
        aruco_pkg = get_package_share_directory('aruco_opencv')
        aruco_param_path = os.path.join(aruco_pkg, 'config', 'aruco_tracker.yaml')
        try:
            with open(aruco_param_path, 'r') as f:
                aruco_yaml = yaml.safe_load(f)
            # Expect structure: {'/**:': {'ros__parameters': {...}}}
            if isinstance(aruco_yaml, dict) and len(aruco_yaml):
                first_key = next(iter(aruco_yaml.keys()))
                aruco_params = aruco_yaml[first_key].get('ros__parameters', {})
            else:
                aruco_params = {}
        except Exception:
            aruco_params = {}

        # Override / normalize the image topic to be relative (remove any leading slash & namespace)
        aruco_params['cam_base_topic'] = 'image_raw'
        # Ensure marker_dict present (fallback)
        aruco_params.setdefault('marker_dict', '5X5_250')

        nodes.append(Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            namespace=ns,
            output='screen',
            parameters=[aruco_params],
            respawn=True
        ))

    return LaunchDescription(nodes)