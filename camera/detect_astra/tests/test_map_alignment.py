"""Stationary laser/map geometry, independent held-out scans and ambiguity."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_map_alignment import MapAlignment,alignment_ok,proposal_issue


class MapAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.grid=np.zeros((150,180),np.int8)
        self.grid[10,10:165]=100;self.grid[135,10:165]=100
        self.grid[10:136,10]=100;self.grid[10:136,165]=100
        self.grid[30:75,130]=100;self.grid[70,95:131]=100
        self.grid[85:115,50]=100
        self.matcher=MapAlignment(self.grid,.05,(-4.,-3.5))
        y,x=np.nonzero(self.grid>=65)
        pts=np.column_stack(((x+.5)*.05-4.,(y+.5)*.05-3.5))
        self.truth=np.array([.4,-.3,.27])
        c,s=np.cos(self.truth[2]),np.sin(self.truth[2])
        self.points=(pts-self.truth[:2])@np.array([[c,-s],[s,c]])

    def test_correct_alignment_and_wrong_yaw_are_distinguished(self):
        self.assertTrue(alignment_ok(self.matcher.metrics(self.truth,self.points)))
        wrong=self.truth+[.1,-.05,math.radians(11)]
        self.assertFalse(alignment_ok(self.matcher.metrics(wrong,self.points)))

    def test_unknown_map_cells_cannot_supply_alignment(self):
        grid=self.grid.copy();grid[:,:]=-1;grid[0,:]=100
        matcher=MapAlignment(grid,.05,(-4.,-3.5))
        self.assertFalse(alignment_ok(matcher.metrics(self.truth,self.points)))

    def test_bounded_proposal_recovers_pose_with_independent_noisy_scan(self):
        wrong=self.truth+[.1,-.05,math.radians(11)]
        solutions=self.matcher.propose(wrong,self.points)
        heldout=self.points+np.random.default_rng(7).normal(0,.005,self.points.shape)
        issue=proposal_issue(self.matcher.metrics(wrong,self.points),solutions,
            self.matcher.metrics(solutions[0]['pose'],heldout))
        self.assertEqual(issue,'')
        p=solutions[0]['pose'];self.assertLess(np.linalg.norm(np.array(p[:2])-self.truth[:2]),.025)
        self.assertLess(abs(p[2]-self.truth[2]),math.radians(1))

    def test_ambiguous_or_weak_or_unimproved_proposal_cannot_reset_pose(self):
        metrics=dict(points=200,median_m=.01,fraction_10cm=.9,score=.001)
        best=dict(pose=[0.,0.,0.],metrics=metrics)
        self.assertTrue(proposal_issue(dict(score=.02),[best],dict(points=200,median_m=.2,fraction_10cm=.3)))
        self.assertTrue(proposal_issue(dict(score=.0011),[best],metrics))
        duplicate=dict(pose=[.3,0.,0.],metrics={**metrics,'score':.0011})
        self.assertTrue(proposal_issue(dict(score=.02),[best,duplicate],metrics))

    def test_insufficient_or_nonfinite_input_is_rejected(self):
        for points in (self.points[:10],np.full((120,2),float('nan'))):
            with self.assertRaises(ValueError):self.matcher.metrics(self.truth,points)


if __name__=='__main__':unittest.main()
