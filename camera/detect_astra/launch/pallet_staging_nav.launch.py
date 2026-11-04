"""Dedicated local Nav2 planner/controller in odom; no AMCL/map or motor drivers."""
from copy import deepcopy
import os
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def staging_parameters():
    shared=Path(get_package_share_directory('tai_robot_one'))/'config/nav2_params.yaml'
    base=yaml.safe_load(shared.read_text())
    planner=deepcopy(base['planner_server']['ros__parameters'])
    planner['use_sim_time']=False
    planner['planner_plugins']=['StagingLattice']
    planner['StagingLattice']=deepcopy(planner['SE2Fallback'])
    for key in ('GridShortest','GridBased','SE2Arc','SE2Fallback'):
        planner.pop(key,None)
    planner['StagingLattice'].update(tolerance=.01,allow_unknown=False,max_planning_time=3.,
                                    analytic_expansion_max_length=1.5)
    controller=deepcopy(base['controller_server']['ros__parameters'])
    controller['use_sim_time']=False
    controller['general_goal_checker'].update(xy_goal_tolerance=.01,yaw_goal_tolerance=.10)
    controller['progress_checker'].update(required_movement_radius=.03,movement_time_allowance=12.)
    controller['FollowPath'].update(
        # Global SafeRotationRPP includes terminal/rear-goal recovery for map
        # navigation. Its 15-cm terminal-turn policy interrupts this very short
        # diagonal before reaching the pallet axis. This dedicated odom action
        # uses stock RPP; staging owns final yaw, no-reverse and full sweeps.
        plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=.10,min_approach_linear_velocity=.10,
        regulated_linear_scaling_min_speed=.10,rotate_to_heading_angular_vel=.22,
        # RPP limits each request from the measured speed. At 20 Hz, 0.3
        # permits only 0.015 rad/s from rest, below the real base deadband;
        # it never starts. Request enough to engage the base; the downstream
        # staging governor still limits acceleration to 0.40 rad/s² and
        # validates measured stopping distance and complete braking sweeps.
        max_angular_accel=2.,min_lookahead_dist=.15,max_lookahead_dist=.30,lookahead_dist=.25,
        # Align to the shallow diagonal before translating. Starting an RPP
        # arc from zero yaw toward a 14-degree bearing can reach ~28 degrees
        # at the endpoint even though every planned pose stays below 15.
        rotate_to_heading_min_angle=.04,allow_reversing=False)
    result={'planner_server':{'ros__parameters':planner},'controller_server':{'ros__parameters':controller}}
    for name in ('local_costmap','global_costmap'):
        params=deepcopy(base[name][name]['ros__parameters'])
        params.update(use_sim_time=False,global_frame='odom',robot_base_frame='base_footprint',
            rolling_window=True,width=8,height=8,resolution=.05,track_unknown_space=True,
            update_frequency=8.,publish_frequency=5.,always_send_full_costmap=True,
            plugins=['lidar_obstacle_layer','inflation_layer'],footprint_padding=.03)
        for key in ('static_layer','camera_obstacle_layer','fork_safety_layer','rear_safety_layer'):
            params.pop(key,None)
        scan=params['lidar_obstacle_layer']['scan']
        scan.update(topic='/scan_filtered',obstacle_max_range=5.5,raytrace_max_range=6.0,
                    expected_update_rate=.3)
        result[name]={name:{'ros__parameters':params}}
    return {'pallet_nav':result}


def generate_launch_description():
    # Nav2's child costmap nodes must receive a namespaced YAML file too.
    handle=tempfile.NamedTemporaryFile(mode='w',prefix='pallet_staging_nav_',suffix='.yaml',delete=False)
    with handle:
        yaml.safe_dump(staging_parameters(),handle)
    params=handle.name
    cpu_mask=os.environ.get('PALLET_COMPUTE_CPUS','')
    if cpu_mask:
        # A numeric mask only; never pass arbitrary environment text to a shell.
        cpu_mask=','.join(str(int(part)) for part in cpu_mask.split(','))
    prefix='taskset --cpu-list '+cpu_mask if cpu_mask else ''
    return LaunchDescription([
        Node(package='nav2_planner',executable='planner_server',name='planner_server',
             namespace='pallet_nav',parameters=[params],output='screen',prefix=prefix),
        Node(package='nav2_controller',executable='controller_server',name='controller_server',
             namespace='pallet_nav',parameters=[params],output='screen',prefix=prefix,
             remappings=[('cmd_vel','/pallet/nav_cmd_vel'),('odom','/odom')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager',
             namespace='pallet_nav',output='screen',parameters=[{
                 'use_sim_time':False,'autostart':True,'bond_timeout':2.,
                 'attempt_respawn_reconnection':False,'node_names':['planner_server','controller_server']}]),
    ])
