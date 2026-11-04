#!/usr/bin/env python3
"""Stationary timing check: actual CM/Nav2/vision, ZERO velocity only.

Requires the existing hardware/localization stack and no active controller.
Never starts the alignment handler, sends FollowPath, or opens a lift writer.
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from run_pallet_cycle import Cycle, ROOT
from align_pallet import MotionGuard, COMMAND
from pallet_cpu_budget import CpuBudget
from pallet_staging_safety import FootprintGrid,braking_poses
from pallet_staging_core import StagingConfig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=60.)
    args=parser.parse_args()
    if not 10<=args.seconds<=120:parser.error('seconds must be between 10 and 120')
    rclpy.init();supervisor=Cycle();probe=Node('pallet_alignment')
    probe.now=lambda:probe.get_clock().now().nanoseconds/1e9
    command=probe.create_publisher(Twist,COMMAND,1)
    guard=MotionGuard(probe)
    rclpy.get_global_executor().add_node(probe)
    supervisor.logs=ROOT/'data/diagnostics'/time.strftime('pallet_stationary_timing_%Y%m%d_%H%M%S')
    supervisor.logs.mkdir(parents=True)
    supervisor.budget=CpuBudget()
    grid=[None];samples=[];checking=[False]
    def receive_grid(msg):
        try:grid[0]=FootprintGrid(msg)
        except ValueError:grid[0]=None
    probe.create_subscription(OccupancyGrid,'/pallet_nav/global_costmap/costmap',receive_grid,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def tick():
        # The only command written anywhere in this diagnostic is all zeros.
        command.publish(Twist())
        ready,reason=guard.ready(probe.now(),time.monotonic())
        if checking[0]:
            age=probe.now()-guard.scan_stamp if guard.scan_stamp is not None else None
            samples.append(dict(t=time.monotonic(),ready=ready,reason=reason,scan_age_s=age))
        # Exercise the same filled-footprint stopping sweeps as the handler,
        # without using their result to generate any movement.
        if grid[0] is not None and 'odom' in supervisor.states:
            od=supervisor.states['odom'][0].pose.pose
            import math
            q=od.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            for pose in braking_poses((od.position.x,od.position.y,yaw),.1,.08,StagingConfig()):
                grid[0].clear(pose)
    timer=probe.create_timer(.05,tick)
    try:
        supervisor.wait(lambda:all(k in supervisor.states for k in ('odom','laser','lift'))
            and supervisor.still() and supervisor.buffer.can_transform('map','base_footprint',rclpy.time.Time()),10.,'stationary hardware')
        supervisor.sensors(empty=True);supervisor.verify_laser_filter()
        if supervisor.active_nav() or supervisor.get_publishers_info_by_topic('/cmd_vel_safe'):
            raise RuntimeError('An existing controller is active; stationary probe refused')
        supervisor.budget.apply_existing()
        supervisor.start('staging',['ros2','launch',str(ROOT/'launch/pallet_alignment_safety.launch.py')])
        supervisor.start('planner',['ros2','launch',str(ROOT/'launch/pallet_staging_nav.launch.py')])
        supervisor.start('perception',[os.environ.get('PALLET_PYTHON',str(ROOT/'.venv/bin/python')),
            str(ROOT/'scripts/run_pallet_perception.py'),'--model',str(ROOT/'data/runtime_models/best_v3_ncnn_model'),
            '--cpu-threads','2','--target-color','green','--measurement-bias-yaml',
            str(ROOT/'config/calibration/pallet_target_bias_2026_11_04_lateral.yaml'),'--heading-floor-reference','--start-paused'])
        watch=('staging','planner','perception')
        supervisor.acquire(watch=watch)
        def ready():
            return guard.ready(probe.now(),time.monotonic())[0] and all(
                w.ready(time.monotonic(),probe.now()) for w in supervisor.sensor_windows.values())
        supervisor.wait(ready,30.,'guard plus continuous five-second sensor window',watch)
        checking[0]=True
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            supervisor.pump()
            supervisor.sensors(empty=True)
            if not supervisor.still():raise RuntimeError('Base is moving; stationary probe stopped')
            if any(supervisor.children[name].poll() is not None for name in watch):
                raise RuntimeError('A supervised component exited')
        bad=[s for s in samples if not s['ready']]
        report=dict(result='PASS' if not bad else 'FAIL',seconds=args.seconds,checks=len(samples),
            invalid_checks=len(bad),invalid_reasons=sorted({s['reason'] for s in bad}),
            max_guard_scan_age_s=max(s['scan_age_s'] for s in samples if s['scan_age_s'] is not None),
            all_velocity_commands_zero=True,lift_commands_sent=False,alignment_handler_started=False,
            base_still=supervisor.still(),lift=supervisor.states['lift'][0],
            recognition=supervisor.fresh('recognition'))
        (supervisor.logs/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
        print('Log:',supervisor.logs,flush=True)
        return 0 if not bad else 1
    finally:
        for name in ('perception','planner','staging'):
            supervisor.stop_child(name)
        supervisor.budget.restore()
        probe.destroy_timer(timer)
        rclpy.get_global_executor().remove_node(probe)
        probe.destroy_node();supervisor.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
