"""ROS-to-HTTP integration test on an isolated ROS domain."""
import json
import math
import os
from pathlib import Path
import sys
import time
import unittest
import urllib.request
import urllib.error
from types import SimpleNamespace

os.environ["ROS_DOMAIN_ID"] = "231"
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from detected_pallets_interfaces.msg import PalletTarget
from test_pallets import (TargetWebSource, WebPreview, TARGET_FRAME,
                          position_from_measurement, alignment_display_reason)


class ForkAlignmentDisplayTest(unittest.TestCase):
    def test_recovered_odometry_does_not_hide_the_current_rgbd_gate(self):
        status=dict(state='LOST_DATA',active=False,ready=False,
                    reason='Odometry không hợp lệ',ready_reason='Chờ RGB-D ổn định 5 giây')
        self.assertEqual(alignment_display_reason(status),status['ready_reason'])
        status['ready']=True
        self.assertIn('Dữ liệu đã phục hồi',alignment_display_reason(status))
        status['ready']=False
        status['active']=True
        self.assertEqual(alignment_display_reason(status),status['reason'])
        status.update(active=False,state='NAV_FAILED',reason='Không tìm được đường')
        self.assertEqual(alignment_display_reason(status),'Không tìm được đường')

    def test_display_preserves_small_angles_and_real_geometry(self):
        msg = SimpleNamespace(chassis_front_forward_m=1.657, opening_lateral_m=-0.049,
                              face_yaw_rad=math.radians(-3.8), entry_heading_rad=math.radians(3.8))
        position = position_from_measurement(msg)
        self.assertEqual(position['fork_bearing_deg'], position['measured_fork_bearing_deg'])
        self.assertAlmostEqual(position['measured_fork_bearing_deg'], math.degrees(math.atan2(-.049, 1.592)))
        self.assertAlmostEqual(position['measured_face_yaw_deg'], -3.8)
        self.assertAlmostEqual(position['plane_alignment_deg'],3.8)
        self.assertAlmostEqual(position['entry_heading_deg'], 3.8)
        self.assertAlmostEqual(position['fork_alignment_deg'], 3.8)
        self.assertAlmostEqual(position['bearing_deg'], math.degrees(math.atan2(-.049, 1.9575)))
        msg.opening_lateral_m = -0.1
        outside = position_from_measurement(msg)
        self.assertLess(outside['fork_bearing_deg'], -2)
        self.assertEqual(outside['fork_bearing_deg'], outside['measured_fork_bearing_deg'])

    def test_rotated_pallet_is_not_aligned_just_because_center_is_ahead(self):
        msg = SimpleNamespace(chassis_front_forward_m=1.7025, opening_lateral_m=-.051,
                              face_yaw_rad=math.radians(8.6), entry_heading_rad=-math.radians(8.6))
        position = position_from_measurement(msg)
        self.assertAlmostEqual(position['fork_bearing_deg'], math.degrees(math.atan2(-.051, 1.6375)))
        self.assertAlmostEqual(position['fork_alignment_deg'], -8.6)
        self.assertAlmostEqual(position['plane_alignment_deg'],8.6)
        self.assertAlmostEqual(position['measured_fork_alignment_deg'], -8.6)
        msg.face_yaw_rad = math.radians(1.4)
        msg.entry_heading_rad = -math.radians(1.4)
        aligned = position_from_measurement(msg)
        self.assertAlmostEqual(aligned['fork_alignment_deg'], -1.4)
        self.assertAlmostEqual(aligned['measured_fork_alignment_deg'], -1.4)
        self.assertEqual(aligned['lateral_m'], -.051)


class TargetWebIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.web = WebPreview("127.0.0.1", 0)
        self.source = TargetWebSource(self.web)
        self.node = rclpy.create_node("pallet_web_test_publisher")
        self.target = self.node.create_publisher(
            PalletTarget, "/pallet/target",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.image = self.node.create_publisher(
            CompressedImage, "/pallet/preview/compressed",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.url = f"http://127.0.0.1:{self.web.server.server_port}"
        self.wait_for(lambda: self.target.get_subscription_count() and self.image.get_subscription_count())

    def tearDown(self):
        self.node.destroy_node()
        self.source.close()
        self.web.close()

    def status(self):
        with urllib.request.urlopen(self.url + "/status.json", timeout=2) as response:
            return json.load(response)

    def wait_for(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.03)
        self.fail("Timed out waiting for ROS/HTTP state")

    def message(self):
        msg = PalletTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = TARGET_FRAME
        msg.valid = True
        msg.camera_forward_m = 1.0
        msg.chassis_front_forward_m = 1.0665
        msg.opening_lateral_m = 0.2
        msg.face_yaw_rad = math.radians(10)
        msg.entry_heading_rad = math.radians(-10)
        return msg

    def publish_with_image(self, msg):
        self.target.publish(msg)
        self.publish_image(msg)

    def publish_image(self, msg):
        image = CompressedImage()
        image.header = msg.header
        image.format = "jpeg"
        image.data = cv2.imencode(".jpg", np.zeros((48, 64, 3), np.uint8))[1].tobytes()
        self.image.publish(image)

    def test_filtered_heading_tolerance_and_invalid_reset(self):
        msg = self.message()
        msg.face_yaw_rad = -math.radians(.4)
        msg.entry_heading_rad = math.radians(.4)
        self.publish_with_image(msg)
        self.wait_for(lambda: self.status()['measurement'] is not None)
        m = self.status()['measurement']
        self.assertAlmostEqual(m['filtered_fork_alignment_deg'], .4)
        self.assertFalse(m['heading_correction_required'])
        self.assertAlmostEqual(m['lateral_m'], .2)  # Within heading tolerance still has lateral error.

        invalid = self.message()
        invalid.valid = False
        self.target.publish(invalid)
        self.wait_for(lambda: self.status()['measurement'] is None)
        msg = self.message()
        msg.face_yaw_rad = math.radians(2.1)
        msg.entry_heading_rad = -math.radians(2.1)
        self.publish_with_image(msg)
        self.wait_for(lambda: self.status()['measurement'] is not None)
        m = self.status()['measurement']
        self.assertAlmostEqual(m['filtered_fork_alignment_deg'], -2.1, places=5)
        self.assertTrue(m['heading_correction_required'])
        self.assertAlmostEqual(m['measured_fork_alignment_deg'], -2.1, places=5)

    def test_detection_images_keep_their_own_versions(self):
        from std_msgs.msg import Bool
        self.source.on_enabled(Bool(data=True))
        images = []
        for level in (45, 180):
            msg = CompressedImage()
            msg.header = self.message().header
            msg.format = 'jpeg'
            msg.data = cv2.imencode('.jpg', np.full((48, 64, 3), level, np.uint8))[1].tobytes()
            images.append(bytes(msg.data))
            self.source.on_detection_image(msg)
        s = self.status()
        self.assertTrue(s['detection_fresh'])
        self.assertEqual(s['detection_frame'], 2)
        for frame, expected in enumerate(images, 1):
            with urllib.request.urlopen(self.url+f'/detection.jpg?frame={frame}') as r:
                self.assertEqual(r.read(), expected)
        self.source.on_enabled(Bool(data=False))
        self.assertFalse(self.status()['detection_fresh'])
        # Recognition images cannot generate or overwrite RGB-D measurements.
        self.assertIsNone(self.status()['measurement'])

    def test_image_first_does_not_blank_or_mix_measurements(self):
        self.publish_with_image(self.message())
        self.wait_for(lambda: self.status()['measurement'] is not None)
        previous = self.status()
        next_target = self.message()
        next_target.chassis_front_forward_m = 1.2
        frame_count = self.source.frame
        self.publish_image(next_target)
        self.wait_for(lambda: self.source.frame > frame_count)
        current = self.status()
        self.assertEqual(current['frame'], previous['frame'])
        self.assertEqual(current['measurement'], previous['measurement'])
        self.target.publish(next_target)
        self.wait_for(lambda: self.status()['frame'] != previous['frame'])
        self.assertAlmostEqual(self.status()['measurement']['forward_m'], 1.5005)
        invalid = self.message()
        invalid.valid = False
        self.target.publish(invalid)
        self.wait_for(lambda: self.status()['measurement'] is None)
        self.publish_image(next_target)
        time.sleep(.08)
        self.assertIsNone(self.status()['measurement'])

    def post_enabled(self, enabled):
        request = urllib.request.Request(
            self.url + '/recognition', data=json.dumps({'enabled': enabled}).encode(),
            headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request, timeout=3) as response:
            return json.load(response)

    def test_alignment_controls_are_separate_and_expire(self):
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        calls = []
        def accept(request, response):
            calls.append(request.data)
            response.success, response.message = True, 'Alignment confirmed'
            return response
        service = self.source.node.create_service(SetBool, '/pallet/alignment/set_enabled', accept)
        publisher = self.node.create_publisher(String, '/pallet/alignment/status', 1)
        try:
            self.wait_for(lambda: publisher.get_subscription_count())
            payload = dict(execute=False, ready=True, active=False, state='IDLE', reason='Ready',
                           ready_reason='', standoff_m=1.2, linear_mps=0., angular_rps=0.,
                           errors=dict(distance_m=.3, lateral_m=.048, heading_deg=1.4))
            publisher.publish(String(data=json.dumps(payload)))
            self.wait_for(lambda: self.status().get('alignment', {}).get('available'))
            self.assertFalse(self.status()['alignment']['execute'])
            self.assertAlmostEqual(self.status()['alignment']['errors']['lateral_m'], .048)
            for enabled in (True, False):
                req = urllib.request.Request(self.url+'/alignment',
                        data=json.dumps({'enabled': enabled}).encode(),
                        headers={'Content-Type': 'application/json'}, method='POST')
                with urllib.request.urlopen(req, timeout=3) as response:
                    self.assertEqual(json.load(response)['message'], 'Alignment confirmed')
            self.assertEqual(calls, [True, False])
            self.assertIsNone(self.status()['recognition']['enabled'])
            payload.update(execute=True,ready=False,active=True,state='LIFTING',
                           strategy='pallet_insertion_odom',reason='Đang nâng càng',
                           standoff_m=.04)
            publisher.publish(String(data=json.dumps(payload)))
            self.wait_for(lambda:self.status().get('alignment',{}).get('state')=='LIFTING')
            self.assertTrue(self.status()['alignment']['available'])
            self.assertTrue(self.status()['alignment']['active'])
            req = urllib.request.Request(self.url+'/alignment',
                    data=json.dumps({'enabled': False}).encode(),
                    headers={'Content-Type': 'application/json'}, method='POST')
            with urllib.request.urlopen(req,timeout=3) as response:
                self.assertEqual(json.load(response)['message'],'Alignment confirmed')
            self.assertEqual(calls,[True,False,False])
            self.wait_for(lambda: not self.status()['alignment']['available'], timeout=2)
            self.assertNotIn('errors', self.status()['alignment'])
            publisher.publish(String(data='{"execute":true}'))
            self.wait_for(lambda: self.source.alignment_status is None)
        finally:
            self.source.node.destroy_service(service)
            self.node.destroy_publisher(publisher)

    def test_alignment_http_waits_for_rgbd_admission_beyond_three_seconds(self):
        import threading
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.task import Future
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
        calls=[];timers=[]
        async def accept(request,response):
            calls.append(request.data)
            if request.data:
                done=Future(executor=self.source.executor)
                timer=threading.Timer(3.5,lambda:done.set_result(True))
                timers.append(timer);timer.start()
                await done
            response.success,response.message=True,'RGB-D ready'
            return response
        service=self.source.node.create_service(SetBool,'/pallet/alignment/set_enabled',accept,
            callback_group=ReentrantCallbackGroup())
        publisher=self.node.create_publisher(String,'/pallet/alignment/status',1)
        try:
            self.wait_for(lambda:publisher.get_subscription_count()
                          and self.source.alignment_client.service_is_ready())
            payload=dict(execute=False,ready=True,active=False,state='IDLE',reason='Ready',
                         ready_reason='',standoff_m=1.2,linear_mps=0.,angular_rps=0.,errors=None)
            publisher.publish(String(data=json.dumps(payload)))
            self.wait_for(lambda:self.status().get('alignment',{}).get('available'))
            request=urllib.request.Request(self.url+'/alignment',
                data=json.dumps({'enabled':True}).encode(),
                headers={'Content-Type':'application/json'},method='POST')
            with urllib.request.urlopen(request,timeout=6) as response:
                self.assertEqual(json.load(response)['message'],'RGB-D ready')
            self.assertEqual(calls,[True])  # No web-generated premature Stop.
        finally:
            for timer in timers:timer.cancel();timer.join()
            self.source.node.destroy_service(service)
            self.node.destroy_publisher(publisher)

    def test_mode_controls_and_inflight_result_rejection(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
        from publish_pallet_detection import DetectionNode
        from detected_pallets_interfaces.msg import PalletDetection
        from sensor_msgs.msg import Image
        target_node = DetectionNode(SimpleNamespace(start_paused=True, max_age_ms=250))
        self.source.executor.add_node(target_node)
        try:
            self.wait_for(lambda: self.status().get('recognition', {}).get('available'))
            self.assertFalse(self.status()['recognition']['enabled'])
            target_node.receive('rgb', Image())
            self.assertFalse(target_node.pending_images['rgb'])
            self.assertFalse(target_node.publish_result(PalletDetection(), 0))

            self.post_enabled(True)
            self.wait_for(lambda: self.status()['recognition']['enabled'] is True)
            prior_search = target_node.semantic_search
            prior_search.window = (160,160,480,400)
            enabled, generation = target_node.inference_state()
            self.assertTrue(enabled)
            target_node.receive('rgb', Image())
            self.assertTrue(target_node.pending_images['rgb'])
            previous = self.message()
            self.publish_with_image(previous)
            self.wait_for(lambda: self.status()['measurement'] is not None)

            self.post_enabled(False)
            self.wait_for(lambda: self.status()['recognition']['enabled'] is False)
            self.assertIsNotNone(self.status()['measurement'])
            # The independent tracker keeps updating while YOLO is paused.
            tracked = self.message()
            tracked.status = 'TRACKED_RGBD'
            tracked.chassis_front_forward_m = 1.12
            self.publish_with_image(tracked)
            self.wait_for(lambda: (m := self.status()['measurement']) is not None
                          and abs(m['forward_m'] - 1.4205) < 1e-6)
            self.assertFalse(target_node.pending_images['rgb'])
            # Simulate inference finishing after pause. It must not republish.
            self.assertFalse(target_node.publish_result(PalletDetection(), generation))
            paused_generation = target_node.inference_state()[1]
            self.post_enabled(False)
            self.assertEqual(target_node.inference_state()[1], paused_generation)

            self.post_enabled(True)
            self.wait_for(lambda: self.status()['recognition']['enabled'] is True)
            self.assertIsNot(target_node.semantic_search,prior_search)
            prior_search.window = (0,0,320,240)  # late result from an old generation
            self.assertIsNone(target_node.semantic_search.window)
            self.assertFalse(target_node.publish_result(PalletDetection(), generation))
            self.wait_for(lambda: self.status()['measurement'] is None)
            self.publish_with_image(self.message())
            self.wait_for(lambda: self.status()['measurement'] is not None)

            with self.assertRaises(urllib.error.HTTPError) as error:
                self.post_enabled('false')
            self.assertEqual(error.exception.code, 400)
            self.assertTrue(target_node.inference_state()[0])

            target_node.enabled_timer.cancel()
            self.source.enabled_received_at = time.monotonic() - 3
            self.source.refresh()
            self.assertIsNone(self.status()['recognition']['enabled'])
            self.assertIsNotNone(self.status()['measurement'])
        finally:
            self.source.spin_stop.set()
            self.source.executor.wake()
            self.source.thread.join(timeout=2)
            self.source.executor.remove_node(target_node)
            target_node.destroy_node()

    def test_coordinates_preview_and_invalid_data(self):
        self.assertIsNone(self.status()["measurement"])
        msg = self.message()
        self.publish_with_image(msg)
        self.wait_for(lambda: self.status()["measurement"] is not None and self.status()["frame"] > 0)
        position = self.status()["measurement"]
        self.assertAlmostEqual(position["forward_m"], 1.367, places=6)
        self.assertAlmostEqual(position["fork_root_forward_m"], 1.0015, places=6)
        self.assertAlmostEqual(position["lateral_m"], 0.2, places=6)
        self.assertAlmostEqual(position["distance_m"], math.hypot(1.367, 0.2), places=6)
        self.assertAlmostEqual(position["bearing_deg"], math.degrees(math.atan2(0.2, 1.367)), places=5)
        self.assertAlmostEqual(position["face_yaw_deg"], 10.0, places=5)
        self.assertAlmostEqual(position["entry_heading_deg"], -10.0, places=5)
        msg.status = 'MEASURED; target_bias_applied'
        self.publish_with_image(msg)
        self.wait_for(lambda: 'Đã áp dụng bù thực nghiệm' in self.status()['measure_status'])
        with urllib.request.urlopen(self.url + f"/frame.jpg?frame={self.status()['frame']}") as response:
            decoded = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (48, 64, 3))

        changed_offset = self.message()
        changed_offset.chassis_front_forward_m = 1.1
        self.publish_with_image(changed_offset)
        self.wait_for(lambda: (m := self.status()["measurement"]) is not None
                      and abs(m["forward_m"] - 1.4005) < 1e-6)

        old_frame = self.status()['frame']
        old_forward = self.status()['measurement']['forward_m']
        unpaired = self.message()
        unpaired.chassis_front_forward_m = 1.2
        self.target.publish(unpaired)
        self.wait_for(lambda: 'Chờ ảnh mới' in self.status()['measure_status'])
        self.assertEqual(self.status()['frame'], old_frame)
        self.assertAlmostEqual(self.status()['measurement']['forward_m'], old_forward)
        self.publish_with_image(unpaired)
        self.wait_for(lambda: (m := self.status()['measurement']) is not None
                      and abs(m['forward_m'] - 1.5005) < 1e-6)

        for reason, edit in (
            ("invalid", lambda m: setattr(m, "valid", False)),
            ("nonfinite", lambda m: setattr(m, "chassis_front_forward_m", math.nan)),
            ("wrong_frame", lambda m: setattr(m.header, "frame_id", "base_link")),
            ("stale", lambda m: setattr(m.header.stamp, "sec", m.header.stamp.sec - 2)),
        ):
            with self.subTest(reason=reason):
                valid = self.message()
                self.publish_with_image(valid)
                self.wait_for(lambda: self.status()["measurement"] is not None)
                bad = self.message()
                edit(bad)
                self.target.publish(bad)
                self.wait_for(lambda: self.status()["measurement"] is None)

        self.publish_with_image(self.message())
        self.wait_for(lambda: self.status()["measurement"] is not None)
        self.wait_for(lambda: self.status()["measurement"] is None, timeout=2)
        cpu = self.status()["cpu"]
        self.assertIsNotNone(cpu["total_percent"])
        self.assertTrue(0 <= cpu["total_percent"] <= 100)
        self.assertEqual(len(cpu["cores"]), cpu["core_count"])


if __name__ == "__main__":
    unittest.main()
