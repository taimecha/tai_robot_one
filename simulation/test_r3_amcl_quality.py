"""A concentrated posterior may pass; ambiguity and excessive spread may not."""
import math
import unittest

from r3_amcl_quality import concentrated_particle_support,validate_runtime_particles


class RuntimeParticleQualityTest(unittest.TestCase):
    def metrics(self):
        return dict(converged=True,xy_rms_m=.007,yaw_std_deg=.1,
                    unique_pose_bins=5,largest_pose_bin_mass=.95)

    def test_low_diversity_is_diagnostic_and_does_not_reject_a_narrow_posterior(self):
        for bins,mass in ((1,1.),(5,.95),(9,.51),(100,.01)):
            metrics=self.metrics();metrics.update(unique_pose_bins=bins,largest_pose_bin_mass=mass)
            validate_runtime_particles(metrics)
            self.assertEqual(concentrated_particle_support(metrics),bins<10 or mass>.5)

    def test_ambiguity_uncertainty_and_invalid_spread_are_still_rejected(self):
        invalid=[None,{},dict(self.metrics(),converged=False),
                 dict(self.metrics(),xy_rms_m=.05),dict(self.metrics(),yaw_std_deg=2.001)]
        for key in ('xy_rms_m','yaw_std_deg'):
            for value in (-.001,math.nan,math.inf,True,None,'0.007'):
                invalid.append(dict(self.metrics(),**{key:value}))
        for metrics in invalid:
            with self.subTest(metrics=metrics),self.assertRaises(ValueError):
                validate_runtime_particles(metrics)
        validate_runtime_particles(dict(self.metrics(),xy_rms_m=.049,yaw_std_deg=2.))


if __name__=='__main__':
    unittest.main()
