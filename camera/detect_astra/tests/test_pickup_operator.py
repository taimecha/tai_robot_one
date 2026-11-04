"""Admission waits use live server health and never bypass ALIGNED/Stop."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_pallet_pickup import insertion_readiness


class OperatorReadinessTest(unittest.TestCase):
    def states(self):
        return dict(handling=({'state':'IDLE','active':False},100.),
            alignment=({'state':'ALIGNED','active':False,'preparing':False},100.),
            recognition=({'phase':'TRACKING','locked':True,'enabled':False,'busy':False,
                'measurement_ready':True,'rgbd_health':{'stable':True}},100.))

    def test_healthy_snapshot_is_ready(self):
        self.assertTrue(insertion_readiness(self.states(),100.1)[0])

    def test_checked_checkpoint_route_can_start_outside_approach_only(self):
        states=self.states();states['alignment'][0]['state']='CHECKPOINT_READY'
        self.assertTrue(insertion_readiness(states,100.1)[0])
        for failed in ('NO_SAFE_GOAL','YAW_CORRECTED','BLOCKED'):
            states['alignment'][0]['state']=failed
            with self.assertRaisesRegex(RuntimeError,'ALIGNED'):insertion_readiness(states,100.1)

    def test_missing_stale_or_future_status_cannot_authorize_start(self):
        for name in self.states():
            for stamp in (98.,101.):
                states=self.states();states[name]=(states[name][0],stamp)
                self.assertFalse(insertion_readiness(states,100.)[0])
            states=self.states();del states[name]
            self.assertFalse(insertion_readiness(states,100.)[0])

    def test_health_measurement_identity_and_yolo_all_gate_start(self):
        for key,value in (('locked',False),('enabled',True),('busy',True),
                          ('measurement_ready',False),('phase','LOST'),('rgbd_health',{'stable':False})):
            states=self.states();states['recognition'][0][key]=value
            self.assertFalse(insertion_readiness(states,100.1)[0],key)

    def test_no_aligned_or_prior_handling_operation_fails_without_retry(self):
        for state in ('IDLE','BLOCKED','STOPPED','LOST_DATA'):
            states=self.states();states['alignment'][0]['state']=state
            with self.assertRaisesRegex(RuntimeError,'ALIGNED'):insertion_readiness(states,100.1)
        states=self.states();states['handling'][0]['state']='STOPPED'
        with self.assertRaisesRegex(RuntimeError,'IDLE'):insertion_readiness(states,100.1)

    def test_only_server_verified_frozen_resume_can_continue_with_clipped_camera(self):
        states=self.states();states['handling'][0].update(state='BLOCKED',frozen_resume_ready=True)
        states['alignment'][0]['state']='BLOCKED'
        states['recognition'][0].update(phase='LOST',locked=False,measurement_ready=False,rgbd_health={'stable':False})
        self.assertTrue(insertion_readiness(states,100.1,recover=True)[0])
        states['recognition'][0]['enabled']=True
        self.assertFalse(insertion_readiness(states,100.1,recover=True)[0])


if __name__=='__main__':unittest.main()
