"""A TF-delayed AMCL pose keeps its exact cloud when a newer one arrives."""
from collections import deque
import copy
import math
from types import SimpleNamespace
import unittest

from r3_amcl_bridge import AmclTrialBridge
from r3_amcl_observation_pair import validate_observation_pair


class CloudHistoryTest(unittest.TestCase):
    def setUp(self):
        self.pose=[3.,1.4,-1.55]
        da=.01
        self.cloud=[(self.pose[0]+x,self.pose[1]+y,self.pose[2]+a,1.)
            for x in (-.01,.01) for y in (-.02,.02) for a in (-da,da)]
        self.cov=[0.]*36
        self.cov[0]=.01**2;self.cov[7]=.02**2;self.cov[35]=-2*math.log(math.cos(da))
        self.wrong=[(x+.02,y,a+.04,w) for x,y,a,w in self.cloud]
        self.probe=SimpleNamespace(latest_cloud=(100.6,self.wrong),
            particle_cloud_history=deque([(100.2,self.cloud),(100.6,self.wrong)]))

    def test_newer_cloud_cannot_starve_a_tf_delayed_matching_pose(self):
        before=copy.deepcopy(self.probe.particle_cloud_history)
        with self.assertRaises(ValueError):
            validate_observation_pair(self.wrong,self.pose,self.cov,100.,100.6,100.7)
        stamp,cloud,proof=AmclTrialBridge.pair_observation_cloud(
            self.probe,self.pose,self.cov,100.,100.7)
        self.assertEqual(stamp,100.2)
        self.assertEqual(cloud,self.cloud)
        self.assertTrue(proof['retained_cloud_posterior_verified'])
        self.assertTrue(proof['full_cloud_state_verified'])
        self.assertEqual(proof['pose_stamp_s'],100.)
        self.assertEqual(self.probe.particle_cloud_history,before)

    def test_history_cannot_substitute_a_different_posterior_or_expired_pair(self):
        for cloud,now in ((self.wrong,100.7),(self.cloud,102.01)):
            with self.subTest(now=now,cloud=cloud):
                self.probe.particle_cloud_history=deque([(100.2,cloud)])
                with self.assertRaises(ValueError):
                    AmclTrialBridge.pair_observation_cloud(
                        self.probe,self.pose,self.cov,100.,now)

    def test_unavailable_latest_stream_does_not_revive_history(self):
        self.probe.latest_cloud=None
        with self.assertRaisesRegex(ValueError,'waiting for global AMCL particle cloud'):
            AmclTrialBridge.pair_observation_cloud(self.probe,self.pose,self.cov,100.,100.7)

    def test_concentrated_latest_cloud_requires_posterior_identity_with_close_stamps(self):
        self.probe.latest_cloud=(100.2,self.cloud)
        stamp,cloud,proof=AmclTrialBridge.pair_observation_cloud(
            self.probe,self.pose,self.cov,100.,100.3)
        self.assertEqual(stamp,100.2)
        self.assertEqual(proof['method'],'posterior_state_agreement')
        self.assertTrue(proof['concentrated_cloud_posterior_verified'])
        self.assertNotIn('retained_cloud_posterior_verified',proof)

        self.probe.latest_cloud=(100.2,self.wrong)
        self.probe.particle_cloud_history.clear()
        with self.assertRaisesRegex(ValueError,'does not reproduce AMCL posterior'):
            AmclTrialBridge.pair_observation_cloud(self.probe,self.pose,self.cov,100.,100.3)

    def test_diverse_latest_cloud_keeps_its_existing_close_timestamp_pairing(self):
        cloud=[(self.pose[0]+i*.001,self.pose[1],self.pose[2],1.) for i in range(20)]
        self.probe.latest_cloud=(100.2,cloud)
        _,_,proof=AmclTrialBridge.pair_observation_cloud(self.probe,self.pose,self.cov,100.,100.3)
        self.assertEqual(proof['method'],'bounded_timestamps')
        self.assertNotIn('concentrated_cloud_posterior_verified',proof)


if __name__=='__main__':
    unittest.main()
