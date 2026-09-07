# Copyright 2026 TAI
# SPDX-License-Identifier: Apache-2.0
"""Regression checks for the directional stop polygon's geometry."""
import ast
from pathlib import Path
import unittest

import yaml


def inside_convex(point, polygon):
    signs = []
    for i, (x, y) in enumerate(polygon):
        nx, ny = polygon[(i+1) % len(polygon)]
        cross = (nx-x)*(point[1]-y)-(ny-y)*(point[0]-x)
        if abs(cross) > 1e-9:
            signs.append(cross > 0)
    return not signs or all(signs) or not any(signs)


class StopEnvelopeTest(unittest.TestCase):
    def setUp(self):
        config = Path(__file__).resolve().parents[1] / 'config/nav2_params.yaml'
        parameters = yaml.safe_load(config.read_text())
        self.polygon = ast.literal_eval(parameters['collision_monitor']['ros__parameters'][
            'VelocityStop']['translation_forward']['points'])
        self.reverse = ast.literal_eval(parameters['collision_monitor']['ros__parameters'][
            'VelocityStop']['translation_reverse']['points'])

    def test_reverse_protects_body_and_rear_stopping_margin(self):
        for point in [(.82, .14), (.82, -.14), (.42, -.29),
                      (-.37, -.29), (-.37, .29), (.42, .29), (-.48, 0)]:
            self.assertTrue(inside_convex(point, self.reverse), point)
        self.assertFalse(inside_convex((.86, 0), self.reverse))

    def test_contains_complete_padded_body(self):
        for point in [(.82, .14), (.82, -.14), (.42, -.29),
                      (-.37, -.29), (-.37, .29), (.42, .29)]:
            self.assertTrue(inside_convex(point, self.polygon), point)

    def test_rear_side_return_does_not_block_departure(self):
        # Actual scan return captured in the reproduced stalled state.
        self.assertFalse(inside_convex((-.074, -.329), self.polygon))

    def test_retains_front_and_rear_collision_protection(self):
        self.assertTrue(inside_convex((.85, .0), self.polygon))
        self.assertTrue(inside_convex((-.3, -.27), self.polygon))


if __name__ == '__main__':
    unittest.main()
