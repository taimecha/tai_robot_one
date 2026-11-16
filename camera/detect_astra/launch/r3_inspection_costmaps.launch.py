"""Stationary RViz costmaps using the existing robot, AMCL and sensor topics.

Only standalone costmap lifecycle nodes are started. There are no navigation
action servers, controllers, behavior servers or velocity command publishers.
"""
import copy
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


COSTMAP_NAMES = ('local_costmap', 'global_costmap')


def inspection_parameters():
    """Reuse real robot geometry while subscribing only to available sensors."""
    root = Path(__file__).resolve().parents[3]
    source = yaml.safe_load((root/'ros2_ws/config/nav2_params.yaml').read_text())
    result = {}
    for name in COSTMAP_NAMES:
        params = copy.deepcopy(source[name][name]['ros__parameters'])
        params.update(use_sim_time=False, always_send_full_costmap=True,
                      update_frequency=3. if name == 'local_costmap' else 2.,
                      publish_frequency=2. if name == 'local_costmap' else 1.,
                      initial_transform_timeout=120.)
        unavailable = {'fork_safety_layer', 'rear_safety_layer'}
        params['plugins'] = [layer for layer in params['plugins'] if layer not in unavailable]
        for layer in unavailable:
            params.pop(layer, None)
        params['lidar_obstacle_layer']['enabled'] = True
        params['lidar_obstacle_layer']['scan']['topic'] = '/scan_filtered'
        params['camera_obstacle_layer']['enabled'] = True
        params['camera_obstacle_layer']['camera_obstacle_scan']['topic'] = '/camera/obstacle_scan'
        if 'static_layer' in params:
            params['static_layer']['map_topic'] = '/map'
            params['static_layer']['map_subscribe_transient_local'] = True
        result[name] = params
    return result


def inspection_actions():
    parameters = inspection_parameters()
    nodes = [
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d',
             namespace=name, name=name, output='screen', parameters=[parameters[name]])
        for name in COSTMAP_NAMES
    ]
    # Start local odom costmap first so it is available while global AMCL is
    # still acquiring map->odom. Global costmap follows when that TF exists.
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='r3_inspection_costmap_manager', output='screen', parameters=[manager_parameters()]))
    return nodes


def manager_parameters():
    return {
        'use_sim_time': False, 'autostart': True,
        'node_names': [f'/{name}/{name}' for name in COSTMAP_NAMES],
        # Standalone Costmap2DROS activation does not create a bond.
        # Waiting for one prevents the manager from activating global.
        'bond_timeout': 0., 'attempt_respawn_reconnection': False,
    }


def generate_launch_description():
    return LaunchDescription(inspection_actions())
