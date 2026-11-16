"""Inspect optional floor cloud wiring without launching camera or robot nodes."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

from r3_floor_aligned_cloud import INPUT_TOPIC, OUTPUT_TOPIC


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('r3_camera_inspection_launch',
    ROOT/'camera/detect_astra/launch/r3_existing_camera_obstacles.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class CameraInspectionLaunchTest(unittest.TestCase):
    def context(self, enabled, report='/tmp/floor alignment/report.json'):
        context = LaunchContext()
        context.launch_configurations.update(
            floor_alignment=str(enabled).lower(), floor_alignment_report=report)
        return context

    def configured_actions(self, enabled, report='/tmp/floor alignment/report.json'):
        # Capture the arguments supplied to real launch constructors. No
        # process or ROS graph is started while checking the sensor pipeline.
        with patch.object(launch, 'Node', side_effect=lambda **kw: dict(kind='node', **kw)), \
                patch.object(launch, 'ExecuteProcess', side_effect=lambda **kw: dict(kind='process', **kw)):
            return launch.obstacle_actions(self.context(enabled, report))

    def assert_single_depth_correction(self, actions):
        nodes = [item for item in actions if item['kind'] == 'node']
        self.assertEqual([node['executable'] for node in nodes],
                         ['publish_rgb_calibrated_cloud.py', 'pointcloud_to_laserscan_node'])
        profiles = [value for node in nodes for value in node['parameters'] if isinstance(value, str)]
        self.assertEqual(profiles, [])
        projection = nodes[0]['parameters'][0]
        self.assertEqual(projection['output_topic'], INPUT_TOPIC)
        self.assertTrue(projection['floor_reference'])
        self.assertEqual(projection['reference_u'], 370.)
        self.assertEqual(projection['max_input_age_s'], .5)
        self.assertEqual(projection['max_rate_hz'], 4.)
        self.assertNotIn('inverse_depth_x_slope', projection)
        self.assertEqual(nodes[0]['additional_env']['OPENBLAS_NUM_THREADS'], '1')
        return nodes

    def test_default_pipeline_remains_two_nodes_without_floor_stage(self):
        actions = self.configured_actions(False)
        self.assertEqual(len(actions), 2)
        nodes = self.assert_single_depth_correction(actions)
        self.assertEqual(dict(nodes[1]['remappings']),
                         {'cloud_in': INPUT_TOPIC, 'scan': '/camera/obstacle_scan'})
        actual = launch.obstacle_actions(self.context(False))
        self.assertTrue(all(isinstance(action, Node) for action in actual))

    def test_floor_inspection_adds_one_worker_after_original_cloud_correction(self):
        report = '/tmp/floor alignment/report.json'
        actions = self.configured_actions(True, report)
        self.assertEqual([action['kind'] for action in actions], ['node', 'process', 'node'])
        nodes = self.assert_single_depth_correction(actions)
        self.assertEqual(actions[1]['cmd'],
                         [sys.executable, str(ROOT/'simulation/r3_floor_aligned_cloud.py'),
                          '--ros-args', '-p', 'report_file:='+report])
        self.assertEqual(dict(nodes[1]['remappings']),
                         {'cloud_in': OUTPUT_TOPIC, 'scan': '/camera/obstacle_scan'})
        actual = launch.obstacle_actions(self.context(True, report))
        self.assertIsInstance(actual[1], ExecuteProcess)
        self.assertNotIsInstance(actual[1], Node)
        self.assertEqual([perform_substitutions(self.context(True), token) for token in actual[1].cmd],
                         actions[1]['cmd'])

    def test_floor_stage_is_explicitly_opted_in(self):
        arguments = {action.name: action for action in launch.generate_launch_description().entities
                     if isinstance(action, DeclareLaunchArgument)}
        self.assertEqual(perform_substitutions(LaunchContext(), arguments['floor_alignment'].default_value),
                         'false')
        self.assertEqual(perform_substitutions(LaunchContext(),
                         arguments['floor_alignment_report'].default_value), '')


if __name__ == '__main__':
    unittest.main()
