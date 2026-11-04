"""Live display geometry and stale-source rejection; no hardware/control nodes."""
import importlib.machinery
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped
from rclpy.time import Time
from rclpy.clock import ClockType

path = Path(__file__).resolve().parents[1]/'scripts/rviz_footprint_display'
loader = importlib.machinery.SourceFileLoader('footprint_display',str(path))
spec = importlib.util.spec_from_loader(loader.name,loader)
display = importlib.util.module_from_spec(spec)
loader.exec_module(display)


class FootprintDisplayTest(unittest.TestCase):
    def test_rotated_world_snapshot_recovers_loaded_body_shape_and_follows_live_tf(self):
        angle = math.radians(-104.4)
        c,s = math.cos(angle),math.sin(angle)
        origin = (2.97,1.46)
        # Rectangular loaded envelope must not revert to an unloaded hexagon.
        points = [(0.84,.32),(.84,-.32),(-.33,-.32),(-.33,.32)]
        message = PolygonStamped()
        message.header.frame_id = 'odom'
        message.polygon.points = [Point32(x=c*x-s*y+origin[0],y=s*x+c*y+origin[1])
                                  for x,y in points]
        t = TransformStamped()
        t.transform.rotation.z = math.sin(-angle/2)
        t.transform.rotation.w = math.cos(-angle/2)
        t.transform.translation.x = -c*origin[0]-s*origin[1]
        t.transform.translation.y = s*origin[0]-c*origin[1]
        marker = display.body_marker(message,t)
        self.assertEqual(len(marker.points),5)
        for point,(x,y) in zip(marker.points,points):
            self.assertAlmostEqual(point.x,x,places=5)
            self.assertAlmostEqual(point.y,y,places=5)
        self.assertEqual(marker.points[-1],marker.points[0])
        self.assertTrue(marker.frame_locked)
        self.assertEqual(marker.header.frame_id,'base_footprint')
        self.assertEqual(marker.header.stamp.sec,0)
        self.assertEqual(marker.header.stamp.nanosec,0)
        self.assertEqual(marker.lifetime.nanosec,800000000)

    def test_empty_or_nonfinite_geometry_never_becomes_live_marker(self):
        message = PolygonStamped();t = TransformStamped();t.transform.rotation.w = 1.
        with self.assertRaises(ValueError):display.body_marker(message,t)
        message.polygon.points = [Point32(x=float('nan'))]*3
        with self.assertRaises(ValueError):display.body_marker(message,t)

    def test_stale_and_unstamped_snapshots_do_not_get_new_lifetime(self):
        for seconds in (0,98,101):
            with self.subTest(seconds=seconds):
                m = PolygonStamped();m.header.frame_id = 'odom';m.header.stamp.sec = seconds
                published=[]
                f = SimpleNamespace(pending=(m,display.time.monotonic()),
                    publisher=SimpleNamespace(publish=published.append),
                    get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=100,clock_type=ClockType.ROS_TIME)))
                display.FootprintDisplay.publish_pending(f)
                self.assertIsNone(f.pending)
                self.assertEqual(published,[])

    def test_transform_uses_snapshot_time_then_marker_expires_from_original_age(self):
        m = PolygonStamped();m.header.frame_id = 'odom'
        m.header.stamp = Time(seconds=99.5).to_msg()
        m.polygon.points = [Point32(x=x,y=y) for x,y in [(0.,0.),(1.,0.),(0.,1.)]]
        t = TransformStamped();t.transform.rotation.w = 1.
        looked=[];published=[]
        def lookup(*args):looked.append(args);return t
        f=SimpleNamespace(pending=(m,display.time.monotonic()),
            publisher=SimpleNamespace(publish=published.append),
            buffer=SimpleNamespace(lookup_transform=lookup),
            get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=100,clock_type=ClockType.ROS_TIME)))
        display.FootprintDisplay.publish_pending(f)
        self.assertEqual(looked[0][:2],('base_footprint','odom'))
        self.assertEqual(looked[0][2].nanoseconds,99500000000)
        self.assertEqual(len(published),1)
        self.assertAlmostEqual(published[0].lifetime.nanosec/1e9,.3)
        self.assertIsNone(f.pending)


if __name__ == '__main__':unittest.main()
