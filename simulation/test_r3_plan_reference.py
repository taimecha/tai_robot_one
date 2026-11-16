"""Freeze actual AMCL-frame plans without relabelling them as wheel odom."""
import copy
import io
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from r3_core import FrozenPlan, PlanReferenceError, compose, errors
from r3_live_prepare import build_plan, provisional_geometry, snapshot_from_samples
import r3_live_prepare as prepare


class PlanReferenceTest(unittest.TestCase):
    def setUp(self):
        self.spec = dict(mode='amcl', run_id='one-parked-trial',
            map_file='/maps/tai_map_real_edited.yaml',
            baseline_map_odom=[3.051, 1.457, math.radians(-90.15)],
            correction_at_capture_xyyaw=[0., 0., 0.])
        # Representative stationary scalars from the reported failing session;
        # these are a deterministic fixture, not replayed raw depth frames.
        self.samples = [dict(stamp=100.+i*.08, odom=[0., 0., 0.],
            forward=1.7527559642791748, lateral=.005598388146609068, raw=-.0194,
            heading=math.radians(.3099259167265581), heading_source='floor_referenced_plane',
            heading_depth_x_slope=.00008, depth_correction_shared=True)
            for i in range(5)]
        self.session = dict(session=7, enabled=False, busy=False, detections=3,
                            yolo_stopped_ros_s=100.2)
        self.snapshot, self.start = snapshot_from_samples(self.samples, self.session)
        self.localized = dict(self.snapshot, frame_id='r3_trial', localization=copy.deepcopy(self.spec))
        self.geometry = provisional_geometry()

    def test_production_build_freezes_amcl_frame_and_preserves_same_odom_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            odom_plan, odom_phases, _ = build_plan(self.snapshot, self.start,
                                                  self.geometry, folder/'odom')
            plan, phases, rows = build_plan(self.localized, self.start,
                                            self.geometry, folder/'amcl')
            target = json.loads(plan.target_json)
            config = json.loads(plan.config_json)
            self.assertEqual(target['frame_id'], 'r3_trial')
            self.assertEqual(target['localization'], self.spec)
            self.assertEqual(config['localization'], self.spec)
            self.assertEqual(target['pose'], self.snapshot['pose'])
            self.assertEqual(phases, odom_phases)
            self.assertTrue(rows)
            self.assertEqual([p['type'] for p in phases], ['APPROACH', 'STRAIGHT'])
            self.assertEqual(json.loads(odom_plan.target_json)['frame_id'], 'odom')
            self.assertNotIn('localization', json.loads(odom_plan.config_json))
            saved = json.loads((folder/'amcl/frozen_plan.json').read_text())
            self.assertEqual(saved['plan_id'], plan.plan_id)
            self.assertEqual(json.loads(saved['target_json']), target)
            # Both existing downstream consumers must accept the frozen frame.
            from r3_localization_contract import localization_spec
            from r3_real_obstacle_filter import read_real_target
            self.assertEqual(localization_spec(target), self.spec)
            accepted, geometry = read_real_target(folder/'amcl/frozen_plan.json', True, max_age_s=None)
            self.assertEqual(accepted, target)
            self.assertEqual(geometry, self.geometry)

    def test_control_status_topic_is_frozen_in_both_target_and_plan_metadata(self):
        self.localized['localization']['control_status_topic']='/pallet/r3/control_odom/status'
        with tempfile.TemporaryDirectory() as directory:
            plan,_,_=build_plan(self.localized,self.start,self.geometry,Path(directory)/'plan')
        target=json.loads(plan.target_json)
        config=json.loads(plan.config_json)
        self.assertEqual(target['localization']['control_status_topic'],'/pallet/r3/control_odom/status')
        self.assertEqual(config['localization'],target['localization'])

    def test_nonzero_trial_frame_keeps_actual_coordinates_and_relative_measurement(self):
        with tempfile.TemporaryDirectory() as directory:
            plan, phases, _ = build_plan(self.localized, self.start, self.geometry,
                                        Path(directory)/'amcl')
        change = (2., -3., .7)
        target = copy.deepcopy(self.localized)
        target['pose'] = list(compose(change, target['pose']))
        shifted = [dict(phase, poses=[compose(change, p) for p in phase['poses']])
                   for phase in phases]
        frozen = FrozenPlan.create(target, self.geometry, shifted, json.loads(plan.config_json))
        self.assertEqual(json.loads(frozen.target_json)['pose'], target['pose'])
        before = errors(self.start, self.localized['pose'], self.geometry)
        after = errors(compose(change, self.start), target['pose'], self.geometry)
        for key in before:
            self.assertAlmostEqual(before[key], after[key])

    def test_bad_reference_fails_before_refinement_without_an_output_directory(self):
        bad_targets = [dict(self.localized, localization=None),
                       dict(self.localized, frame_id='map'),
                       dict(self.localized, localization=dict(self.spec, run_id='')),
                       dict(self.localized, localization=dict(self.spec,
                            correction_at_capture_xyyaw=[0., math.nan, 0.])),
                       dict(self.snapshot, localization=self.spec)]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'plan'
            for target in bad_targets:
                with self.subTest(target=target), \
                        patch.object(prepare, 'minimum_jerk_approach') as refine:
                    with self.assertRaises(PlanReferenceError):
                        build_plan(target, self.start, self.geometry, output)
                    refine.assert_not_called()
                    self.assertFalse(output.exists())

    def test_config_must_match_target_reference_before_path_validation(self):
        for target, config in (
            (self.localized, {}),
            (self.localized, dict(localization=dict(self.spec, run_id='another-session'))),
            (self.snapshot, dict(localization=self.spec)),
        ):
            with self.subTest(config=config), patch('r3_core.validate_phases') as validate:
                with self.assertRaisesRegex(PlanReferenceError, 'metadata differs'):
                    FrozenPlan.create(target, self.geometry, [], config)
                validate.assert_not_called()

    def test_main_reports_reference_error_once_instead_of_requesting_more_images(self):
        node = SimpleNamespace(status=self.session, status_wall=time.monotonic(),
            last_target_wall=time.monotonic(), yolo_off_wall=time.monotonic(),
            samples=self.samples, localization_status=dict(ready=True, frame_id='r3_trial',
                run_id=self.spec['run_id'], map_file=self.spec['map_file'],
                published_ros_s=100.4, baseline_map_odom=self.spec['baseline_map_odom'],
                correction_xyyaw=[0., 0., 0.], correction_settled=True),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100400000000)),
            destroy_node=Mock())
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, \
                patch('sys.argv', ['r3_live_prepare.py', '--output', str(Path(directory)/'plan'),
                                   '--localization', 'amcl']), \
                patch.object(prepare, 'Capture', return_value=node), \
                patch.object(prepare, 'build_plan', side_effect=PlanReferenceError('bad run reference')) as build, \
                patch.object(prepare.rclpy, 'init'), patch.object(prepare.rclpy, 'shutdown'), \
                patch.object(prepare.rclpy, 'ok', return_value=True), \
                patch.object(prepare.rclpy, 'spin_once'), patch('sys.stdout', output):
            result = prepare.main()
        self.assertEqual(result, 2)
        build.assert_called_once()
        report = json.loads(output.getvalue())
        self.assertEqual(report['status'], 'BLOCKED')
        self.assertIn('frame/reference', report['reason'])
        self.assertEqual(report['robot_commands'], 0)
        self.assertNotIn('REACQUIRE', output.getvalue())
        node.destroy_node.assert_called_once()


if __name__ == '__main__':
    unittest.main()
