"""Independent map-fit gate tests; no ROS messages or motion publishers."""
import math
import random
import unittest
from copy import deepcopy
from types import SimpleNamespace as NS

from r3_scan_map_check import ScanMapCheck


def map_fixture(yaw=0.):
    width,height,resolution = 120,200,.05
    data=[0]*(width*height)
    for y in range(height):
        data[y*width+80]=100
    return NS(info=NS(width=width,height=height,resolution=resolution,
                      origin=NS(position=NS(x=-1.,y=-5.),
                                orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),
              data=data)


def scan_fixture(count=121):
    increment=1./(count-1)
    return NS(angle_min=-.5,angle_increment=increment,range_min=.1,range_max=10.,
              ranges=[3.025/math.cos(-.5+i*increment) for i in range(count)])


class ScanMapCheckTest(unittest.TestCase):
    def isolated_cells(self,cells=((10,10),),yaw=0.,resolution=.05):
        width=height=30
        data=[0]*(width*height)
        for x,y in cells:data[y*width+x]=100
        return NS(info=NS(width=width,height=height,resolution=resolution,
            origin=NS(position=NS(x=-.2,y=.1),orientation=NS(
                x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),data=data)

    def world_point(self,grid,x,y):
        q=grid.info.origin.orientation;yaw=2*math.atan2(q.z,q.w)
        origin=grid.info.origin.position
        return origin.x+math.cos(yaw)*x-math.sin(yaw)*y,origin.y+math.sin(yaw)*x+math.cos(yaw)*y

    def test_endpoint_distance_measures_wall_area_not_cell_to_cell_steps(self):
        grid=self.isolated_cells();check=ScanMapCheck(grid)
        self.assertEqual(check.endpoint_distance(*self.world_point(grid,.525,.525)),0.)
        self.assertAlmostEqual(check.endpoint_distance(*self.world_point(grid,.4995,.525)),.0005,places=12)
        self.assertAlmostEqual(check.endpoint_distance(*self.world_point(grid,.5505,.525)),.0005,places=12)
        self.assertAlmostEqual(check.endpoint_distance(*self.world_point(grid,.58,.59)),.05,places=12)

    def test_rotated_map_cell_area_uses_local_square_bounds(self):
        grid=self.isolated_cells(yaw=.73);check=ScanMapCheck(grid)
        self.assertAlmostEqual(check.endpoint_distance(*self.world_point(grid,.4995,.525)),.0005,places=12)
        self.assertAlmostEqual(check.endpoint_distance(*self.world_point(grid,.58,.59)),.05,places=12)
        self.assertEqual(check.endpoint_distance(*self.world_point(grid,.525,.525)),0.)

    def test_multiwall_area_distances_match_independent_bruteforce_reference(self):
        cells=((2,3),(8,9),(11,3),(15,15),(3,14),(10,17))
        grid=self.isolated_cells(cells,yaw=-.41);check=ScanMapCheck(grid)
        rng=random.Random(412)
        for index in range(300):
            # Sample near multiple isolated walls, including between their
            # nearest-source regions. The reference visits every wall square.
            cx,cy=cells[index%len(cells)]
            x=max(.001,min(1.499,(cx+.5)*.05+rng.uniform(-.2,.2)))
            y=max(.001,min(1.499,(cy+.5)*.05+rng.uniform(-.2,.2)))
            expected=min(math.hypot(max(sx*.05-x,0.,x-(sx+1)*.05),
                                   max(sy*.05-y,0.,y-(sy+1)*.05)) for sx,sy in cells)
            actual=check.endpoint_distance(*self.world_point(grid,x,y))
            with self.subTest(index=index,point=(x,y)):
                self.assertAlmostEqual(actual,expected,places=12)

    def test_float32_resolution_does_not_exclude_nominal_fifteen_cm_boundary(self):
        resolution=.05000000074505806
        grid=self.isolated_cells(resolution=resolution);check=ScanMapCheck(grid)
        right=11*resolution;middle=10.5*resolution
        scan=NS(angle_min=0.,angle_increment=1e-12,range_min=.01,range_max=2.,ranges=[.1]*31)
        pose=self.world_point(grid,right+.15-.1,middle)
        result=check.score(scan,(*pose,0.))
        self.assertEqual(result['inlier_endpoints'],31)
        farther=self.world_point(grid,right+.1501-.1,middle)
        self.assertEqual(check.score(scan,(*farther,0.))['inlier_endpoints'],0)
        self.assertEqual(result['inlier_distance_m'],.15)

    def test_unknown_and_bounded_far_queries_still_cannot_claim_wall_support(self):
        grid=self.isolated_cells();grid.data[10*grid.info.width+9]=-1
        check=ScanMapCheck(grid)
        self.assertEqual(check.endpoint_distance(*self.world_point(grid,.4995,.525)),math.inf)
        self.assertEqual(check.endpoint_distance(*self.world_point(grid,1.2,1.2)),math.inf)

    def test_correct_wall_pose_passes_but_translated_or_rotated_pose_fails(self):
        check=ScanMapCheck(map_fixture())
        scan=scan_fixture()
        result=check.score(scan,(0.,0.,0.))
        self.assertTrue(result['ready'])
        self.assertEqual(result['sampled_endpoints'],120)
        self.assertAlmostEqual(result['inlier_ratio'],1.)
        self.assertFalse(result['uniqueness_verified'])
        for pose in ((.5,0.,0.), (0.,0.,math.pi/2), (10.,0.,0.)):
            with self.subTest(pose=pose):
                self.assertFalse(check.score(scan,pose)['ready'])

    def test_rotated_map_origin_is_respected(self):
        yaw=.7
        # The unrotated scanner is at local grid coordinates (1,5).
        pose=(-1.+math.cos(yaw)-5.*math.sin(yaw),
              -5.+math.sin(yaw)+5.*math.cos(yaw),yaw)
        result=ScanMapCheck(map_fixture(yaw)).score(scan_fixture(),pose)
        self.assertTrue(result['ready'])
        self.assertAlmostEqual(result['inlier_ratio'],1.)

    def test_noise_and_minority_moving_obstacles_are_tolerated(self):
        scan=scan_fixture()
        scan.ranges=[value+(.025 if i%2 else -.025) for i,value in enumerate(scan.ranges)]
        for i in range(0,len(scan.ranges),5):
            scan.ranges[i]=1.
        result=ScanMapCheck(map_fixture()).score(scan,(0.,0.,0.))
        self.assertTrue(result['ready'])
        self.assertGreater(result['inlier_ratio'],.75)
        # The same total sample count cannot rescue a majority mismatch.
        for i in range(0,len(scan.ranges),2):
            scan.ranges[i]=1.
        self.assertFalse(ScanMapCheck(map_fixture()).score(scan,(0.,0.,0.))['ready'])

    def test_invalid_and_max_range_rays_do_not_claim_map_agreement(self):
        scan=scan_fixture()
        scan.ranges=[math.inf,math.nan,0.,-1.,scan.range_max,scan.range_max+1.]*30
        result=ScanMapCheck(map_fixture()).score(scan,(0.,0.,0.))
        self.assertFalse(result['ready'])
        self.assertEqual(result['valid_endpoints'],0)
        self.assertIsNone(result['median_distance_m'])
        scan.ranges=[3.025]*29
        self.assertFalse(ScanMapCheck(map_fixture()).score(scan,(0.,0.,0.))['ready'])

    def test_unknown_map_endpoints_and_outside_map_do_not_count_as_inliers(self):
        grid=map_fixture()
        check=ScanMapCheck(grid)
        self.assertEqual(check.endpoint_distance(3.025,0.),0.)
        for y in range(80,121):
            grid.data[y*grid.info.width+80]=-1
        check=ScanMapCheck(grid)
        self.assertEqual(check.endpoint_distance(3.025,0.),math.inf)
        self.assertEqual(check.endpoint_distance(100.,100.),math.inf)
        self.assertFalse(check.score(scan_fixture(),(0.,0.,0.))['ready'])

    def test_invalid_map_and_scan_are_rejected(self):
        grid=map_fixture()
        grid.info.resolution=0.
        with self.assertRaises(ValueError):
            ScanMapCheck(grid)
        grid=map_fixture()
        grid.data=[0]*len(grid.data)
        with self.assertRaisesRegex(ValueError,'no occupied'):
            ScanMapCheck(grid)
        check=ScanMapCheck(map_fixture())
        scan=scan_fixture()
        scan.angle_increment=0.
        with self.assertRaises(ValueError):
            check.score(scan,(0.,0.,0.))

    def test_trusted_sectors_follow_90_degree_mounting_in_base_coordinates(self):
        check = ScanMapCheck(map_fixture())
        scan = scan_fixture()
        result = check.score_trusted_sectors(scan, (0., 0., 0.), (0., 0., math.pi / 2))
        # Scanner-forward endpoints become robot-left with a +90 degree mount.
        self.assertTrue(result['robot_left']['ready'])
        self.assertEqual(result['robot_left']['valid_endpoints'], len(scan.ranges))
        self.assertEqual(result['robot_left']['sampled_endpoints'], 60)
        self.assertTrue(result['rear_left']['ready'])
        expected = sum(-r * math.sin(scan.angle_min + i * scan.angle_increment) < -.3
                       for i, r in enumerate(scan.ranges))
        self.assertEqual(result['rear_left']['valid_endpoints'], expected)
        self.assertEqual(result['selection_frame'], 'base_footprint')
        self.assertFalse(result['used_for_pose_refinement'])
        self.assertFalse(result['physical_accuracy_verified'])

    def test_trusted_sectors_follow_180_degree_mounting(self):
        check = ScanMapCheck(map_fixture())
        scan = scan_fixture()
        result = check.score_trusted_sectors(scan, (0., 0., 0.), (0., 0., math.pi))
        expected = sum(-r * math.sin(scan.angle_min + i * scan.angle_increment) > .25
                       for i, r in enumerate(scan.ranges))
        self.assertGreater(expected, 15)
        self.assertLess(expected, len(scan.ranges))
        # With a reversed scanner, negative raw angles belong to robot-left.
        for name in ('robot_left', 'rear_left'):
            self.assertTrue(result[name]['ready'])
            self.assertEqual(result[name]['valid_endpoints'], expected)
            self.assertEqual(result[name]['inlier_ratio'], 1.)
            self.assertFalse(result[name]['uniqueness_verified'])

    def test_sensor_translation_changes_sector_membership(self):
        check = ScanMapCheck(map_fixture())
        scan = scan_fixture()
        original = check.score_trusted_sectors(scan, (0., 0., 0.), (0., 0., 0.))
        shifted_y = check.score_trusted_sectors(scan, (0., 0., 0.), (0., 2., 0.))
        shifted_xy = check.score_trusted_sectors(scan, (0., 0., 0.), (-4., 2., 0.))
        self.assertLess(original['robot_left']['valid_endpoints'], len(scan.ranges))
        self.assertEqual(original['rear_left']['valid_endpoints'], 0)
        self.assertEqual(shifted_y['robot_left']['valid_endpoints'], len(scan.ranges))
        self.assertEqual(shifted_y['rear_left']['valid_endpoints'], 0)
        self.assertEqual(shifted_xy['rear_left']['valid_endpoints'], len(scan.ranges))
        self.assertTrue(shifted_xy['rear_left']['ready'])

    def test_mismatched_opposite_side_beams_do_not_count_in_trusted_fit(self):
        check = ScanMapCheck(map_fixture())
        scan = scan_fixture()
        baseline = check.score_trusted_sectors(scan, (0., 0., 0.), (0., 0., math.pi))
        # Positive scanner angles face robot-right for this mounting. Shorten
        # those returns so the overall fit fails but trusted-left fit is intact.
        for i in range(61, len(scan.ranges)):
            scan.ranges[i] = 1.
        result = check.score_trusted_sectors(scan, (0., 0., 0.), (0., 0., math.pi))
        self.assertFalse(check.score(scan, (0., 0., 0.))['ready'])
        for name in ('robot_left', 'rear_left'):
            self.assertEqual(result[name]['valid_endpoints'], baseline[name]['valid_endpoints'])
            self.assertEqual(result[name]['inlier_ratio'], 1.)
            self.assertTrue(result[name]['ready'])

    def test_trusted_diagnostics_do_not_mutate_scan_or_change_overall_score(self):
        check = ScanMapCheck(map_fixture())
        scan = scan_fixture()
        original = deepcopy(scan.__dict__)
        overall = check.score(scan, (0., 0., 0.))
        mounting = [0.1, .2, math.pi / 2]
        original_mounting = list(mounting)
        check.score_trusted_sectors(scan, (0., 0., 0.), mounting)
        self.assertEqual(scan.__dict__, original)
        self.assertEqual(mounting, original_mounting)
        self.assertEqual(check.score(scan, (0., 0., 0.)), overall)

    def test_invalid_trusted_sector_mounting_is_rejected(self):
        check = ScanMapCheck(map_fixture())
        for mounting in ((0., 0.), (0., math.nan, 0.), (0., 0., math.inf)):
            with self.subTest(mounting=mounting):
                with self.assertRaisesRegex(ValueError, 'base/sensor'):
                    check.score_trusted_sectors(scan_fixture(), (0., 0., 0.), mounting)


if __name__=='__main__':
    unittest.main()
