"""Staging/pickup only; existing hardware, camera and AMCL remain running."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    token=os.environ.get('PALLET_CYCLE_TOKEN')
    if not token:raise RuntimeError('Existing-hardware staging requires the cycle heartbeat token')
    handler=ExecuteProcess(cmd=[os.environ.get('PALLET_PYTHON',str(root/'.venv/bin/python')),
        str(root/'scripts/handle_pallet.py'),'--execute','--cycle-token',token,
        *(['--usable-depth-m',os.environ['PALLET_USABLE_DEPTH_M']]
          if os.environ.get('PALLET_USABLE_DEPTH_M') else [])],output='screen')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/pallet_alignment_safety.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/pallet_staging_nav.launch.py'))),
        RegisterEventHandler(OnProcessExit(target_action=handler,on_exit=[
            EmitEvent(event=Shutdown(reason='Cycle staging owner exited'))])),handler])
