from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_pickup_admission import SensorWindow


class SensorWindowTest(unittest.TestCase):
    def test_requires_continuous_capture_and_receipt_before_start(self):
        w=SensorWindow(.2)
        for i in range(101):
            t=i*.05;w.observe(100+t,t,100+t)
            self.assertEqual(w.ready(t,100+t),i==100)

    def test_gap_stale_capture_and_replays_do_not_extend_readiness(self):
        for kind in ('gap','stale','replay'):
            w=SensorWindow(.2,settle_s=.5)
            for i in range(11):w.observe(100+i*.05,i*.05,100+i*.05)
            self.assertTrue(w.ready(.5,100.5))
            if kind=='gap':w.observe(100.8,.8,100.8)
            elif kind=='stale':w.observe(100.6,.8,101.)
            else:w.observe(100.5,.8,100.8)
            self.assertFalse(w.ready(.8,100.8))


if __name__=='__main__':unittest.main()
