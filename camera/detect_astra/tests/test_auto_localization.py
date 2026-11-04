"""AMCL global qualification: multimodality, fresh windows, invalid inputs."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_auto_localization import particle_issue,PoseStability,summary_observation,navigation_stability_window


class AutoLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.pose=(1.,2.,.1);self.cov=np.zeros(36);self.cov[[0,7,35]]=[.003,.003,.001]
        self.cloud=np.tile([*self.pose,1/1000],(1000,1))

    def test_single_cluster_with_small_covariance_qualifies(self):
        self.assertEqual(particle_issue(self.pose,self.cov,self.cloud,True),'')

    def test_collapsed_cloud_covariance_roundoff_is_not_negative_variance(self):
        self.cov[[0,7,35]]=[-1e-12,1e-12,-1e-13]
        self.assertFalse(particle_issue(self.pose,self.cov,self.cloud,True))
        self.cov[0]=-.001
        self.assertTrue(particle_issue(self.pose,self.cov,self.cloud,True))

    def test_second_cluster_is_rejected_even_when_best_covariance_is_small(self):
        self.cloud[:250,0]+=2.
        self.assertIn('giả thuyết',particle_issue(self.pose,self.cov,self.cloud,True))

    def test_bad_or_broad_cloud_and_covariance_are_rejected(self):
        for column,value in ((0,float('nan')),(3,-.001),(3,0.)):
            c=self.cloud.copy();c[:,column]=value
            self.assertTrue(particle_issue(self.pose,self.cov,c))
        self.cov[0]=.05
        self.assertTrue(particle_issue(self.pose,self.cov,self.cloud))
        self.assertTrue(particle_issue(self.pose,np.zeros(36),self.cloud[:10]))

    def test_strict_gate_is_tighter_than_coarse(self):
        self.cov[0]=.15**2
        self.assertFalse(particle_issue(self.pose,self.cov,self.cloud))
        self.assertTrue(particle_issue(self.pose,self.cov,self.cloud,True))

    def test_stationary_window_needs_five_seconds_new_stamps_and_wall_time(self):
        window=PoseStability()
        for i in range(20):self.assertFalse(window.observe(i*.25,i*.25,self.pose,True))
        self.assertTrue(window.observe(5.,5.,self.pose,True))
        self.assertFalse(window.observe(5.,20.,self.pose,True))
        window=PoseStability()
        for i in range(21):self.assertFalse(window.observe(i*.25,i*.01,self.pose,True))

    def test_relaxed_navigation_accepts_two_seconds_with_bounded_jitter(self):
        window=navigation_stability_window()
        for i in range(9):
            # 4.5 cm and 2.5 degrees of stationary estimator jitter: inside
            # the new bounds, outside the old 3 cm / 2 degree window.
            pose=(1.+.045*(i%2),2.,.1+math.radians(2.5)*(i%2))
            self.assertEqual(window.observe(i*.25,i*.25,pose,True),i==8)
        self.assertFalse(window.observe(2.,3.,self.pose,True))

    def test_relaxed_hold_still_requires_wall_time_and_five_distinct_samples(self):
        window=navigation_stability_window()
        for i in range(9):self.assertFalse(window.observe(i*.25,i*.05,self.pose,True))
        window=navigation_stability_window()
        for t in (0.,.7,1.4,2.1):self.assertFalse(window.observe(t,t,self.pose,True))
        self.assertTrue(window.observe(2.2,2.2,self.pose,True))

    def test_relaxed_hold_rejects_excess_jitter_invalid_data_and_long_gaps(self):
        for distance,angle in ((.051,0.),(0.,math.radians(3.1))):
            window=navigation_stability_window()
            for i in range(9):
                pose=(1.+distance*(i%2),2.,.1+angle*(i%2))
                self.assertFalse(window.observe(i*.25,i*.25,pose,True))
        for failure in ('invalid','source_gap','wall_gap'):
            window=navigation_stability_window()
            for i in range(6):self.assertFalse(window.observe(i*.25,i*.25,self.pose,True))
            if failure=='invalid':
                self.assertFalse(window.observe(1.5,1.5,self.pose,False))
                self.assertFalse(window.observe(2.,2.,self.pose,True))
            elif failure=='source_gap':self.assertFalse(window.observe(2.5,2.,self.pose,True))
            else:self.assertFalse(window.observe(2.,2.5,self.pose,True))

    def test_invalid_samples_gap_and_drift_cannot_qualify(self):
        for failure in ('invalid','gap','drift'):
            window=PoseStability()
            for i in range(21):
                t=i*.25;pose=self.pose
                if failure=='invalid' and i==10:window.observe(t,t,pose,False);continue
                if failure=='gap' and i in (8,9,10,11,12):continue
                if failure=='drift':pose=(self.pose[0]+i*.004,*self.pose[1:])
                self.assertFalse(window.observe(t,t,pose,True),failure)

    def test_worker_summary_keeps_source_age_even_with_fresh_publication(self):
        summary=dict(valid=True,pose=[1.,2.,.1],pose_stamp_s=100.,cloud_stamp_s=100.,
                     published_ros_s=100.1,compute_s=.02,particle_count=12000,
                     coarse_issue='',strict_issue='needs convergence')
        self.assertEqual(summary_observation(summary,99.,100.2),(100.,(1.,2.,.1),''))
        self.assertEqual(summary_observation(summary,99.,100.2,True)[2],'needs convergence')
        summary['published_ros_s']=101.
        self.assertIsNone(summary_observation(summary,99.,101.1)[0])
        self.assertIsNone(summary_observation(summary,100.01,100.2)[0])

    def test_invalid_or_missing_worker_summary_cannot_qualify(self):
        for summary in ({},{'valid':False,'reason':'worker failed'},
                        {'valid':True,'pose':[float('nan'),0.,0.]}):
            self.assertIsNone(summary_observation(summary,99.,100.2)[0])

    def test_heading_window_handles_wraparound(self):
        w=PoseStability()
        for i in range(21):
            result=w.observe(i*.25,i*.25,(0.,0.,math.pi-.002 if i%2 else -math.pi+.002),True)
        self.assertTrue(result)


if __name__=='__main__':unittest.main()
