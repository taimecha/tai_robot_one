"""Real DDS action discovery in domain 227; no goals or hardware commands."""
import os
from pathlib import Path
import sys
import time
import unittest

os.environ['ROS_DOMAIN_ID']='227'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.action import ActionClient,ActionServer
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose
from run_pallet_delivery import wait_for_unique_nav2


class DeliveryDiscoveryRosTest(unittest.TestCase):
    def test_cold_start_waits_and_duplicate_real_action_endpoints_are_rejected(self):
        rclpy.init()
        executor=SingleThreadedExecutor()
        probe=rclpy.create_node('delivery_discovery_test')
        first=rclpy.create_node('bt_navigator')
        second=rclpy.create_node('bt_navigator')
        nodes=(probe,first,second)
        for node in nodes:executor.add_node(node)
        servers=[];executed=[]
        client=ActionClient(probe,NavigateToPose,'/navigate_to_pose')
        def execute(goal):
            executed.append(goal)
            goal.succeed()
            return NavigateToPose.Result()
        def start():
            timer.cancel()
            servers.append(ActionServer(first,NavigateToPose,'/navigate_to_pose',execute))
        timer=first.create_timer(.7,start)
        def pump(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        try:
            owner=wait_for_unique_nav2(probe,client,pump,timeout=8.)
            self.assertEqual(len(owner),16)
            servers.append(ActionServer(second,NavigateToPose,'/navigate_to_pose',execute))
            end=time.monotonic()+5.
            while (len(probe.get_publishers_info_by_topic('/navigate_to_pose/_action/status'))!=2
                   and time.monotonic()<end):pump(.05)
            self.assertEqual(len(probe.get_publishers_info_by_topic('/navigate_to_pose/_action/status')),2)
            with self.assertRaisesRegex(RuntimeError,'status_publishers='):
                wait_for_unique_nav2(probe,client,pump,timeout=1.5)
            self.assertEqual(executed,[])
            self.assertFalse(probe.get_publishers_info_by_topic('/cmd_vel_safe'))
        finally:
            for server in servers:server.destroy()
            client.destroy()
            for node in nodes:executor.remove_node(node);node.destroy_node()
            executor.shutdown();rclpy.shutdown()


if __name__=='__main__':unittest.main()
