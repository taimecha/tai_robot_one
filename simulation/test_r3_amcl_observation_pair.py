"""Jazzy scan/publisher timestamp differences need bounded state agreement."""
import copy
import math
import unittest

from r3_amcl_observation_pair import validate_observation_pair,validate_pair_evidence


class AmclObservationPairTest(unittest.TestCase):
    def setUp(self):
        self.reference=[1.,2.,math.pi-.003]
        angle=math.radians(.5)
        self.particles=[]
        for dx in (-.01,.01):
            for dy in (-.02,.02):
                for da in (-angle,angle):
                    yaw=math.atan2(math.sin(self.reference[2]+da),math.cos(self.reference[2]+da))
                    self.particles.append((1.+dx,2.+dy,yaw,1.))
        self.covariance=[0.]*36
        self.covariance[0]=.01**2
        self.covariance[7]=.02**2
        self.covariance[35]=-2*math.log(math.cos(angle))

    def pair(self,**changes):
        args=dict(particles=self.particles,reference_xyyaw=self.reference,
                  cov36=self.covariance,pose_stamp=100.,cloud_stamp=100.6,now=100.7)
        args.update(changes)
        return validate_observation_pair(**args)

    def snapshot(self,proof):
        return dict(amcl_stamp_s=100.,cloud_stamp_s=100.6,
                    map_pose_xyyaw=list(self.reference),covariance_6x6=list(self.covariance),
                    particle_pairing=proof)

    def test_close_timestamps_keep_legacy_pairing_without_claiming_full_state_proof(self):
        proof=self.pair(particles=[],cloud_stamp=100.2,now=100.3)
        self.assertEqual(proof['method'],'bounded_timestamps')
        self.assertFalse(proof['full_cloud_state_verified'])
        self.assertTrue(proof['timestamps_fresh_at_pairing'])
        self.assertFalse(proof['physical_accuracy_verified'])
        snapshot=self.snapshot(proof);snapshot['cloud_stamp_s']=100.2
        self.assertEqual(validate_pair_evidence(snapshot),proof)

    def test_equal_posterior_accepts_bounded_publication_lag_and_wrapped_yaw(self):
        before=copy.deepcopy((self.particles,self.reference,self.covariance))
        proof=self.pair()
        self.assertEqual(proof['method'],'posterior_state_agreement')
        self.assertTrue(proof['full_cloud_state_verified'])
        self.assertEqual(proof['pose_stamp_s'],100.)
        self.assertEqual(proof['cloud_stamp_s'],100.6)
        self.assertAlmostEqual(proof['cloud_pose_lag_s'],.6)
        self.assertLess(proof['residuals']['mean_distance_m'],1e-12)
        self.assertLess(proof['residuals']['mean_yaw_difference_deg'],1e-10)
        self.assertEqual(validate_pair_evidence(self.snapshot(proof)),proof)
        self.assertEqual((self.particles,self.reference,self.covariance),before)

    def test_retained_close_time_cloud_still_requires_numeric_state_identity(self):
        proof=self.pair(cloud_stamp=100.2,now=100.7,require_posterior_identity=True)
        self.assertEqual(proof['method'],'posterior_state_agreement')
        snapshot=self.snapshot(proof);snapshot['cloud_stamp_s']=100.2
        self.assertEqual(validate_pair_evidence(snapshot),proof)
        shifted=[(x+.02,y,a,w) for x,y,a,w in self.particles]
        with self.assertRaisesRegex(ValueError,'mean does not reproduce'):
            self.pair(particles=shifted,cloud_stamp=100.2,now=100.7,
                      require_posterior_identity=True)

    def test_timestamp_bounds_apply_even_when_posterior_state_matches(self):
        cases=[dict(cloud_stamp=102.001,now=102.001),
               dict(cloud_stamp=99.949,now=100.1),dict(now=102.001),
               dict(cloud_stamp=100.8,now=100.7),dict(pose_stamp=math.nan),
               dict(pose_stamp=-1.,cloud_stamp=-.5,now=0.)]
        for changes in cases:
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.pair(**changes)
        self.assertEqual(self.pair(cloud_stamp=102.,now=102.)['method'],
                         'posterior_state_agreement')

    def test_different_posterior_cannot_be_paired_using_lag_alone(self):
        shifted=[(x+.0011,y,a,w) for x,y,a,w in self.particles]
        yaw_shifted=[(x,y,a+math.radians(.011),w) for x,y,a,w in self.particles]
        covariance=list(self.covariance);covariance[0]*=1.02
        for changes in (dict(particles=shifted),dict(particles=yaw_shifted),
                        dict(cov36=covariance)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.pair(**changes)

    def test_ambiguous_full_cloud_is_rejected_despite_equal_mean(self):
        particles=[(.6,2.,self.reference[2],1.),(1.4,2.,self.reference[2],1.)]
        covariance=list(self.covariance);covariance[0]=.4**2;covariance[7]=covariance[35]=0.
        with self.assertRaisesRegex(ValueError,'ambiguous'):
            self.pair(particles=particles,cov36=covariance)

    def test_nonzero_xy_cross_covariance_is_checked(self):
        particles=[(.99,1.98,self.reference[2],1.),(1.01,2.02,self.reference[2],1.)]
        covariance=list(self.covariance);covariance[1]=covariance[6]=.0002;covariance[35]=0.
        self.pair(particles=particles,cov36=covariance)
        covariance[1]=covariance[6]=-.0002
        with self.assertRaisesRegex(ValueError,'covariance does not reproduce'):
            self.pair(particles=particles,cov36=covariance)

    def test_invalid_cloud_covariance_and_nonfinite_inputs_are_rejected(self):
        covariance=list(self.covariance);covariance[1]=math.nan
        cases=[dict(particles=[]),dict(particles=[(1.,2.,0.,0.)]),
               dict(particles=[(1.,2.,0.,-1.)]),dict(particles=[(math.nan,2.,0.,1.)]),
               dict(cov36=covariance),dict(cov36=None),dict(reference_xyyaw=[1.,2.,math.inf])]
        for changes in cases:
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.pair(**changes)

    def test_reader_rejects_missing_proof_for_delayed_snapshot(self):
        snapshot=self.snapshot(self.pair());snapshot.pop('particle_pairing')
        with self.assertRaisesRegex(ValueError,'needs original posterior'):
            validate_pair_evidence(snapshot)
        snapshot['cloud_stamp_s']=100.2
        self.assertEqual(validate_pair_evidence(snapshot)['method'],'bounded_timestamps')

    def test_reader_recomputes_evidence_instead_of_trusting_claimed_success(self):
        snapshot=self.snapshot(self.pair())
        cases=[]
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['cloud_state']['mean_xyyaw'][0]+=.02
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['cloud_state']['covariance']['xx']*=1.05
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['cloud_state']['dominant_mass']=.7
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['residuals']['mean_distance_m']=.01
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['residuals']['covariance_absolute_differences']['xx']=.1
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['validated_ros_s']=103.
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['pose_stamp_s']=99.9
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['cloud_pose_lag_s']=.2
        cases.append(bad)
        bad=copy.deepcopy(snapshot);bad['particle_pairing']['method']='bounded_timestamps'
        cases.append(bad)
        for bad in cases:
            with self.subTest(proof=bad['particle_pairing']),self.assertRaises(ValueError):
                validate_pair_evidence(bad)
        self.assertEqual(validate_pair_evidence(snapshot),snapshot['particle_pairing'])


if __name__=='__main__':
    unittest.main()
