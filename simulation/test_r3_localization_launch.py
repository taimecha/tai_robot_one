"""Localization-only launch contract; no ROS nodes or hardware are started."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT/'camera/detect_astra/launch/r3_localization.launch.py'
spec = importlib.util.spec_from_file_location('r3_localization_launch', LAUNCH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LocalizationLaunchTest(unittest.TestCase):
    def configured_nodes(self, stationary=None, precision=None):
        context = LaunchContext()
        context.launch_configurations['map_file'] = str(ROOT/'maps/tai_warehouse.yaml')
        if stationary is not None:
            context.launch_configurations['stationary_inspection'] = stationary
        if precision is not None:
            context.launch_configurations['precision_localization'] = precision
        # Inspect the actual constructor arguments without starting ROS nodes.
        with patch.object(module, 'Node', side_effect=lambda **kw: kw):
            return module.localization_actions(context)

    def test_missing_or_unselected_map_is_not_replaced_by_a_guessed_map(self):
        for invalid in ('', 'maps/tai_warehouse.yaml', '/missing/map.yaml'):
            with self.subTest(path=invalid), self.assertRaises(ValueError):
                module.validate_map_file(invalid)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'map.yaml'
            path.write_text('image: missing.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
            with self.assertRaisesRegex(ValueError, 'image does not exist'):
                module.validate_map_file(path)

    def test_selected_map_with_relative_image_is_supported(self):
        path = ROOT/'maps/tai_warehouse.yaml'
        self.assertEqual(module.validate_map_file(path), path)
        context = LaunchContext()
        context.launch_configurations['map_file'] = str(path)
        nodes = module.localization_actions(context)
        # Exact executable set protects the existing trial's command/TF owners.
        self.assertEqual([node.node_package for node in nodes],
                         ['nav2_map_server', 'nav2_amcl', 'nav2_lifecycle_manager'])
        self.assertEqual([node.node_executable for node in nodes],
                         ['map_server', 'amcl', 'lifecycle_manager'])

    def test_amcl_requires_explicit_initial_pose_and_reuses_existing_odom_scan(self):
        config = yaml.safe_load((ROOT/'camera/detect_astra/config/r3_localization.yaml').read_text())
        amcl = config['amcl']['ros__parameters']
        self.assertFalse(amcl['set_initial_pose'])
        self.assertTrue(amcl['always_reset_initial_pose'])
        self.assertNotIn('initial_pose', amcl)
        self.assertEqual(amcl['scan_topic'], '/scan_filtered')
        self.assertEqual((amcl['global_frame_id'], amcl['odom_frame_id'], amcl['base_frame_id']),
                         ('map', 'odom', 'base_footprint'))
        self.assertTrue(amcl['tf_broadcast'])
        self.assertFalse(amcl['use_sim_time'])

    def test_base_profile_preserves_original_trial_likelihood_and_particle_limits(self):
        amcl = yaml.safe_load((ROOT/'camera/detect_astra/config/r3_localization.yaml').read_text())['amcl']['ros__parameters']
        self.assertEqual(amcl['laser_model_type'], 'likelihood_field')
        self.assertEqual(amcl['max_beams'], 120)
        self.assertEqual(amcl['min_particles'], 500)
        self.assertEqual(amcl['max_particles'], 2000)
        self.assertNotIn('do_beamskip', amcl)
        self.assertNotIn('sigma_hit', amcl)

    def test_default_and_explicit_false_select_only_base_amcl_file(self):
        base = str(ROOT/'camera/detect_astra/config/r3_localization.yaml')
        for stationary in (None, 'false', 'FALSE', '0'):
            with self.subTest(stationary=stationary):
                nodes = self.configured_nodes(stationary)
                self.assertEqual(nodes[1]['parameters'], [base])
                self.assertEqual(nodes[0]['parameters'], [base, {
                    'yaml_filename': str(ROOT/'maps/tai_warehouse.yaml')}])
        arguments = {action.name: action for action in module.generate_launch_description().entities
                     if isinstance(action, DeclareLaunchArgument)}
        self.assertEqual(perform_substitutions(LaunchContext(),
                         arguments['stationary_inspection'].default_value), 'false')

    def test_stationary_profile_applies_partial_override_to_amcl_only(self):
        base = ROOT/'camera/detect_astra/config/r3_localization.yaml'
        override = base.with_name('r3_stationary_localization.yaml')
        expected = {
            'laser_model_type': 'likelihood_field_prob', 'sigma_hit': .10,
            'z_hit': .90, 'z_rand': .10, 'do_beamskip': True,
            'beam_skip_distance': .25, 'beam_skip_threshold': .30,
            'beam_skip_error_threshold': .90, 'max_beams': 120,
            'min_particles': 2000, 'max_particles': 4000,
        }
        contents = yaml.safe_load(override.read_text())
        self.assertEqual(set(contents), {'amcl'})
        self.assertEqual(contents['amcl']['ros__parameters'], expected)
        for stationary in ('true', 'TRUE', '1'):
            with self.subTest(stationary=stationary):
                nodes = self.configured_nodes(stationary)
                self.assertEqual(nodes[1]['parameters'], [str(base), str(override)])
                self.assertEqual(nodes[0]['parameters'], [str(base), {
                    'yaml_filename': str(ROOT/'maps/tai_warehouse.yaml')}])
                # Merge the selected files to verify precedence and retained
                # frame/motion settings, rather than only checking a flag.
                merged = {}
                for selected in nodes[1]['parameters']:
                    merged.update(yaml.safe_load(Path(selected).read_text())['amcl']['ros__parameters'])
                for key, value in expected.items():
                    self.assertEqual(merged[key], value)
                self.assertEqual(merged['scan_topic'], '/scan_filtered')
                self.assertEqual(merged['update_min_d'], .03)

    def test_stationary_boolean_rejects_ambiguous_values(self):
        for invalid in ('', 'yes', 'maybe', '2'):
            with self.subTest(value=invalid):
                with self.assertRaisesRegex(ValueError, 'stationary_inspection'):
                    self.configured_nodes(invalid)

    def test_navigation_precision_selects_same_sensor_model_without_inspection(self):
        nodes=self.configured_nodes(stationary='false',precision='true')
        base=ROOT/'camera/detect_astra/config/r3_localization.yaml'
        self.assertEqual(nodes[1]['parameters'],[str(base),str(base.with_name('r3_stationary_localization.yaml'))])
        self.assertEqual(nodes[0]['parameters'][0],str(base))
        for invalid in ('yes','maybe','2',''):
            with self.subTest(precision=invalid),self.assertRaisesRegex(ValueError,'precision_localization'):
                self.configured_nodes(precision=invalid)


if __name__ == '__main__':
    unittest.main()
