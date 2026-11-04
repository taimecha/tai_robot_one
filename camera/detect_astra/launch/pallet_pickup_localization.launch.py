"""AMCL + map only: no drivers, saved pose restore, or velocity publishers."""
import os
from copy import deepcopy
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project=Path(__file__).resolve().parents[1]
    robot=Path(get_package_share_directory('tai_robot_one'))
    params=yaml.safe_load((robot/'config/nav2_params.yaml').read_text())
    amcl=deepcopy(params['amcl']['ros__parameters'])
    amcl.update(use_sim_time=False,set_initial_pose=False,always_reset_initial_pose=False,
                scan_topic='/localization/scan',min_particles=1000,max_particles=12000,max_beams=120,
                update_min_d=.03,update_min_a=.02,alpha2=.02,alpha3=.05,alpha4=.05)
    return LaunchDescription([
        DeclareLaunchArgument('map_file',default_value=str(project.parents[1]/'ros2_ws/maps/tai_map_real_edited.yaml')),
        ExecuteProcess(cmd=[os.environ.get('PALLET_PYTHON',str(project/'.venv/bin/python')),
            str(project/'scripts/pallet_localization_scan.py')],output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
             parameters=[{'use_sim_time':False,'yaml_filename':LaunchConfiguration('map_file')}]),
        Node(package='nav2_amcl',executable='amcl',name='amcl',output='screen',parameters=[amcl]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='pallet_localization_manager',
             output='screen',parameters=[{'use_sim_time':False,'autostart':True,
                 'bond_timeout':4.,'attempt_respawn_reconnection':False,'node_names':['map_server','amcl']}]),
    ])
