"""Read-only LiDAR costmap and overlays; no controller or motor publishers."""
import importlib.util
from pathlib import Path
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('pallet_staging_nav_view',root/'launch/pallet_staging_nav.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    params=module.staging_parameters()['pallet_nav']
    # A planner owns its costmap but cannot send velocity commands. Do not
    # start the controller, Collision Monitor, base driver or pallet handler.
    params.pop('controller_server');params.pop('local_costmap')
    with tempfile.NamedTemporaryFile(mode='w',prefix='pallet_obstacle_view_',suffix='.yaml',delete=False) as file:
        yaml.safe_dump({'pallet_view':params},file);parameter_file=file.name
    return LaunchDescription([
        Node(package='nav2_planner',executable='planner_server',namespace='pallet_view',
             parameters=[parameter_file],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',namespace='pallet_view',
             name='lifecycle_manager_view',parameters=[dict(use_sim_time=False,autostart=True,
                 node_names=['planner_server'],bond_timeout=2.,attempt_respawn_reconnection=False)]),
        ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scripts/pallet_obstacle_view.py')],output='screen'),
    ])
