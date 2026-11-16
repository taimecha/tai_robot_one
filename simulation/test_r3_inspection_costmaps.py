"""Stationary costmap launch keeps sensor geometry without command servers."""
import importlib.util
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('r3_inspection_costmaps',
    ROOT/'camera/detect_astra/launch/r3_inspection_costmaps.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class InspectionCostmapsTest(unittest.TestCase):
    def test_costmaps_use_real_available_sensors_and_preserve_geometry(self):
        source = yaml.safe_load((ROOT/'ros2_ws/config/nav2_params.yaml').read_text())
        parameters = launch.inspection_parameters()
        self.assertEqual(set(parameters), {'global_costmap', 'local_costmap'})
        for name, params in parameters.items():
            with self.subTest(costmap=name):
                original = source[name][name]['ros__parameters']
                for key in ('footprint', 'footprint_padding', 'resolution', 'global_frame',
                            'robot_base_frame', 'inflation_layer'):
                    self.assertEqual(params[key], original[key])
                self.assertFalse(params['use_sim_time'])
                self.assertTrue(params['always_send_full_costmap'])
                self.assertTrue(params['camera_obstacle_layer']['enabled'])
                self.assertTrue(params['lidar_obstacle_layer']['enabled'])
                self.assertEqual(params['lidar_obstacle_layer']['scan']['topic'], '/scan_filtered')
                self.assertEqual(params['camera_obstacle_layer']['camera_obstacle_scan']['topic'],
                                 '/camera/obstacle_scan')
                self.assertNotIn('fork_safety_layer', params['plugins'])
                self.assertNotIn('rear_safety_layer', params['plugins'])
                self.assertNotIn('fork_safety_layer', params)
                self.assertNotIn('rear_safety_layer', params)
        self.assertEqual(parameters['global_costmap']['static_layer']['map_topic'], '/map')
        self.assertTrue(parameters['global_costmap']['static_layer']['map_subscribe_transient_local'])

    def test_only_costmaps_and_lifecycle_manager_are_launched(self):
        nodes = launch.inspection_actions()
        self.assertEqual([node.node_package for node in nodes],
                         ['nav2_costmap_2d', 'nav2_costmap_2d', 'nav2_lifecycle_manager'])
        self.assertEqual([node.node_executable for node in nodes],
                         ['nav2_costmap_2d', 'nav2_costmap_2d', 'lifecycle_manager'])
        self.assertEqual(launch.COSTMAP_NAMES, ('local_costmap', 'global_costmap'))
        manager = launch.manager_parameters()
        self.assertEqual(manager['node_names'],
                         ['/local_costmap/local_costmap', '/global_costmap/global_costmap'])
        self.assertEqual(manager['bond_timeout'], 0.)


if __name__ == '__main__':
    unittest.main()
