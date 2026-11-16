"""Explain the actual failing poses before an operator authorizes Nav2."""
import math
import unittest

from r3_navigation_preview import preview_lines


class NavigationPreviewTest(unittest.TestCase):
    def report(self, pose):
        return dict(status='READY_FOR_NAVIGATION', check_only=True,
            navigation_goal_attempted=False, current_pose_xyyaw=pose,
            goal_pose_xyyaw=[3.051, 1.457, -1.5734143206728881])

    def test_small_lateral_offset_inside_recognition_region_needs_no_positioning_turn(self):
        text='\n'.join(preview_lines(self.report(
            [3.0044223321027146,1.4251870732510317,-1.5589379774050767])))
        self.assertIn('5.64 cm',text)
        self.assertIn('-0.83°',text)
        self.assertIn('+123.7°',text)
        self.assertIn('phía sau',text)
        self.assertNotIn('Nav2 có thể phải xoay',text)
        self.assertIn('không chỉnh thêm',text)
        self.assertIn('Nhận diện tự bắt đầu',text)

    def test_coincident_position_has_no_invented_bearing_turn(self):
        text='\n'.join(preview_lines(self.report([3.051,1.457,math.radians(-80.)])))
        self.assertIn('cách 0.00 cm',text)
        self.assertIn('phương tới A lệch +0.0°',text)
        self.assertNotIn('Nav2 có thể phải xoay',text)
        self.assertIn('Nav2 chỉ căn lại hướng tại chỗ',text)

    def test_failed_check_or_nonfinite_pose_never_produces_a_motion_preview(self):
        valid=self.report([3.,1.4,-1.5])
        for invalid in (dict(valid,status='BLOCKED'),
                dict(valid,navigation_goal_attempted=True),
                dict(valid,current_pose_xyyaw=[math.nan,1.,0.])):
            with self.subTest(report=invalid), self.assertRaises(ValueError):
                preview_lines(invalid)


if __name__=='__main__':
    unittest.main()
