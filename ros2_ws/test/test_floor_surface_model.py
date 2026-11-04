"""Ground qualification must not learn a pallet/low platform as the floor."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from floor_surface_model import estimate_floor_residual,FloorSurfaceModel


def scene(plane=(0.,.045,.012)):
    x,y = np.meshgrid(np.linspace(1.2,2.7,65),np.linspace(-1.,1.,65))
    z = plane[0]*x+plane[1]*y+plane[2]
    return np.stack((x,y,z),axis=-1).astype(np.float32)


class FloorSurfaceTest(unittest.TestCase):
    def test_residual_tilt_removed_but_six_cm_obstacle_and_pallet_retained(self):
        plane=np.array([-.006,.077,.025])
        floor=scene(plane);rng=np.random.default_rng(18)
        observed=floor.copy();observed[:,:,2]+=rng.normal(0.,.003,floor.shape[:2])
        model=FloorSurfaceModel()
        for i in range(12):model.observe(observed,i*.51,True)
        self.assertIsNotNone(model.plane)
        for height in (.065,.10):
            obstacle=floor.copy();obstacle[:,:,2]+=height*np.sqrt(1+sum(plane[:2]**2))
            corrected=model.apply(obstacle)
            self.assertTrue((corrected[:,:,2]>=.06).all())
            np.testing.assert_array_equal(corrected[:,:,:2],obstacle[:,:,:2])
        self.assertLess(abs(model.apply(observed)[:,:,2]).max(),.015)

    def test_full_frame_pallet_or_low_platform_never_qualifies(self):
        for height in (.04,.06,.08,.10,.15):
            with self.subTest(height=height):
                self.assertIsNone(estimate_floor_residual(scene((0.,0.,height))))

    def test_pallet_patch_does_not_replace_broad_floor(self):
        plane=np.array([-.005,.015,.015]);points=scene(plane)
        pallet=(points[:,:,0]>1.8)&(points[:,:,0]<2.2)&(abs(points[:,:,1]-.35)<.2)
        points[pallet,2]+=.10
        fitted=estimate_floor_residual(points)
        self.assertIsNotNone(fitted)
        np.testing.assert_allclose(fitted,plane,atol=1e-6)

    def test_dominant_low_obstacle_blocks_new_fit_instead_of_becoming_ground(self):
        points=scene((0.,0.,0.))
        points[points[:,:,0]<2.4,2]+=.065
        self.assertIsNone(estimate_floor_residual(points))

    def test_small_one_sided_missing_or_excessively_tilted_surface_is_rejected(self):
        a=scene()
        for points in (a[:10,:10],a[a[:,:,1]>.1],np.full_like(a,np.nan),scene((0.,.18,0.))):
            self.assertIsNone(estimate_floor_residual(points))

    def test_model_requires_continuous_stationary_stable_window(self):
        model=FloorSurfaceModel();floor=scene()
        for i in range(7):model.observe(floor,i*.51,True)
        self.assertIsNone(model.plane)
        model.observe(floor,3.6,False)
        self.assertEqual(len(model.samples),0)
        for i in range(7):model.observe(floor,4.+i*.51,True)
        self.assertIsNone(model.plane)
        model.observe(floor,8.,True) # Gap restarts qualification.
        self.assertEqual(len(model.samples),1)
        self.assertIsNone(model.plane)

    def test_moving_or_obscured_view_freezes_last_model_and_preserves_invalids(self):
        model=FloorSurfaceModel();floor=scene()
        for i in range(10):model.observe(floor,i*.51,True)
        previous=model.plane.copy()
        for i in range(10):model.observe(scene((0.,0.,.025)),6.+i*.51,False)
        np.testing.assert_array_equal(model.plane,previous)
        model.observe(scene((0.,0.,.10)),12.,True)
        np.testing.assert_array_equal(model.plane,previous)
        floor[0,0]=np.nan
        self.assertTrue(np.isnan(model.apply(floor)[0,0]).all())

    def test_drifting_candidates_cannot_commit_an_unstable_plane(self):
        model=FloorSurfaceModel()
        for i in range(12):model.observe(scene((0.,.015,.01 if i%2 else -.01)),i*.51,True)
        self.assertIsNone(model.plane)


if __name__=='__main__':unittest.main()
