"""Route checks must not starve odom; late/stale results cannot open motion."""
from array import array
from concurrent.futures import ThreadPoolExecutor
from copy import copy
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_pallet as stage
from builtin_interfaces.msg import Time
from nav_msgs.msg import Path as NavPath
from pallet_alignment_core import OdomSample
from pallet_staging_core import StagingController
from pallet_staging_safety import FootprintGrid


def grid():
    msg=NS(header=NS(frame_id='odom'),info=NS(resolution=.05,width=160,height=160,
        origin=NS(position=NS(x=-4.,y=-4.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=array('b',[0]*25600))
    return FootprintGrid(msg)


class Fixture:
    schedule_route_validation=stage.PalletStagingNode.schedule_route_validation
    poll_route_validation=stage.PalletStagingNode.poll_route_validation
    cancel_actions=stage.PalletStagingNode.cancel_actions
    def __init__(self):
        self.core=StagingController();self.core.active=True;self.core.state='PLANNING'
        self.core.goal=(.3,.05,0.);self.core.frozen_target=(2.,.05,0.)
        self.grid=grid();self.grid_stamp=1.;self.route_pool=ThreadPoolExecutor(max_workers=1)
        self.route_job=self.route_input=self.route_retry_deadline=None
        self.followed=[];self.zeros=[];self.handles=[];self.future=None
        self.pending_grid_path=None;self.cancel_pending=[];self.checked_heading_bounds=None
        self.refresh()
    def now(self):return time.monotonic()
    def refresh(self):
        self.last_odom_wall=time.monotonic()
        self.core.odom.add(OdomSample(self.now(),0.,0.,0.))
    def get_clock(self):return NS(now=lambda:NS(to_msg=lambda:Time(sec=int(self.now()))))
    def interlock(self):return (self.now()-self.last_odom_wall<=.2,'Chưa có odometry mới')
    def follow_checked_route(self,path):self.followed.append(path);self.operation='follow_accept'
    def zero(self):self.zeros.append(True)
    def get_logger(self):return NS(info=lambda *a:None,warning=lambda *a:None)
    def path(self):
        p=NavPath();p.header.frame_id='odom'
        p.poses=[stage.route_pose((0.,0.,0.),Time()),stage.route_pose(self.core.goal,Time())]
        return p
    def wait_job(self):
        end=time.monotonic()+2
        while not self.route_job.done() and time.monotonic()<end:
            self.refresh();time.sleep(.01)
        if not self.route_job.done():raise RuntimeError('test worker timed out')
    def close(self):self.route_pool.shutdown(wait=True,cancel_futures=True)


class RouteWorkerTest(unittest.TestCase):
    def test_slow_geometry_leaves_parent_free_to_receive_fresh_odom(self):
        f=Fixture();original=stage.check_route_snapshot
        def slow(snapshot,path):
            end=time.monotonic()+.35
            while time.monotonic()<end:math.sin(time.monotonic())
            return original(snapshot,path)
        try:
            with patch.object(stage,'check_route_snapshot',slow):
                started=time.monotonic();f.schedule_route_validation(f.path())
                self.assertLess(time.monotonic()-started,.10)
                callbacks=0
                while not f.route_job.done():
                    f.refresh();callbacks+=1
                    self.assertTrue(f.interlock()[0]);time.sleep(.015)
                self.assertGreater(callbacks,5)
                f.refresh();f.poll_route_validation()
                self.assertEqual(len(f.followed),1)
        finally:f.close()

    def test_stale_odom_result_is_rejected_before_follow(self):
        f=Fixture()
        try:
            f.schedule_route_validation(f.path());f.wait_job()
            f.last_odom_wall-=.25;f.poll_route_validation()
            self.assertFalse(f.followed);self.assertEqual(f.core.state,'BLOCKED')
        finally:f.close()

    def test_cancel_discards_late_worker_without_follow(self):
        f=Fixture()
        try:
            f.schedule_route_validation(f.path());job=f.route_job
            f.core.stop();f.cancel_actions()
            if not job.cancelled():job.result(timeout=2)
            f.poll_route_validation();self.assertFalse(f.followed)
            self.assertIsNone(f.route_job)
        finally:f.close()

    def test_new_obstacle_forces_revalidation_and_never_reuses_old_approval(self):
        f=Fixture()
        try:
            f.schedule_route_validation(f.path());f.wait_job()
            old=f.grid;fresh=copy(old);fresh.data=array('b',old.data);fresh.costs=None
            fresh.data[80*160+85]=100;f.grid=fresh;f.grid_stamp+=1
            self.assertFalse(stage.same_obstacles(old,fresh))
            f.refresh();f.poll_route_validation()
            self.assertFalse(f.followed);self.assertIsNotNone(f.route_job)
            f.cancel_actions()
        finally:f.close()

    def test_base_displacement_during_validation_rejects_approval(self):
        f=Fixture()
        try:
            f.schedule_route_validation(f.path());f.wait_job()
            f.core.odom.add(OdomSample(f.now(),.01,0.,0.));f.last_odom_wall=f.now()
            f.poll_route_validation();self.assertFalse(f.followed)
            self.assertEqual(f.core.state,'BLOCKED')
        finally:f.close()


if __name__=='__main__':unittest.main()
