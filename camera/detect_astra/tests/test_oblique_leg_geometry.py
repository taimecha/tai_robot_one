"""Complete front legs may have different image heights under perspective."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_target_geometry import _leg_components


class ObliqueLegTest(unittest.TestCase):
    def mask(self,right_bottom=172):
        mask=np.zeros((240,320),bool)
        mask[81:149,64:219]=True
        mask[149:190,65:94]=True
        mask[149:right_bottom,200:219]=True
        return mask,(64,81,219,190)

    def test_both_complete_perspective_feet_are_kept_with_common_sampling_rows(self):
        mask,bbox=self.mask()
        legs=_leg_components(mask,bbox)
        self.assertEqual(len(legs),2)
        self.assertLess(legs[1]['bottom'],legs[0]['bottom']-8)
        self.assertGreaterEqual(min(l['bottom'] for l in legs)-max(l['top'] for l in legs),8)

    def test_one_foot_or_high_floating_patch_is_not_a_complete_pair(self):
        for bottom in (149,155):
            mask,bbox=self.mask(bottom)
            with self.assertRaises(ValueError):_leg_components(mask,bbox)

    def test_joined_lower_mask_cannot_supply_two_front_legs(self):
        mask,bbox=self.mask();mask[149:190,64:219]=True
        with self.assertRaises(ValueError):_leg_components(mask,bbox)


if __name__=='__main__':unittest.main()
