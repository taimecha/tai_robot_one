"""Lifecycle readiness failures and bounded retries; no ROS nodes or hardware."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from r3_wait_localization_active import LifecyclePoll, NAMES, readiness, wait_active


def future(state=3, done=True, error=None):
    result=Mock(return_value=SimpleNamespace(current_state=SimpleNamespace(id=state)),side_effect=error)
    return SimpleNamespace(done=lambda:done,result=result)


class LocalizationActiveTest(unittest.TestCase):
    def clients(self):
        return {name:Mock(service_is_ready=Mock(return_value=True),
                          call_async=Mock(return_value=future())) for name in NAMES}

    def test_both_states_must_be_fresh_active_successes(self):
        states={name:dict(id=3,stamp=10.) for name in NAMES}
        self.assertTrue(readiness(states,10.1)[0])
        for change in ({},dict(id=2,stamp=10.),dict(id=3,stamp=7.),dict(id=3,stamp=11.),
                       dict(id=3,stamp=math.nan),dict(id=3,stamp=10.,error='service failed')):
            with self.subTest(change=change):
                ready,reason=readiness(dict(states,amcl=change),10.1)
                self.assertFalse(ready)
                self.assertIn('amcl',reason)
        self.assertFalse(readiness({},10.)[0])

    def test_independent_services_can_recover_from_discovery_and_send_errors(self):
        clients=self.clients()
        clients['amcl'].service_is_ready.side_effect=[False,True,True]
        clients['amcl'].call_async.side_effect=[RuntimeError('async_send_request failed'),future()]
        poll=LifecyclePoll(clients)
        poll.tick(0.);poll.tick(.1)
        self.assertEqual(poll.states['map_server']['id'],3)
        self.assertIn('unavailable',poll.states['amcl']['error'])
        poll.tick(.3)
        self.assertIn('async_send_request failed',poll.states['amcl']['error'])
        poll.tick(.6);poll.tick(.7)
        self.assertTrue(readiness(poll.states,.7)[0])

    def test_failed_response_and_timed_out_request_are_retried(self):
        clients=self.clients()
        blocked=future(done=False)
        clients['amcl'].call_async.side_effect=[blocked,future(error=RuntimeError('response failed')),future()]
        poll=LifecyclePoll(clients)
        poll.tick(0.);poll.tick(2.01)
        clients['amcl'].remove_pending_request.assert_called_once_with(blocked)
        self.assertIn('timed out',poll.states['amcl']['error'])
        poll.tick(2.3);poll.tick(2.4)
        self.assertIn('response failed',poll.states['amcl']['error'])
        poll.tick(2.7);poll.tick(2.8)
        self.assertTrue(readiness(poll.states,2.8)[0])

    def test_wait_has_total_wallclock_bound_with_unavailable_service(self):
        clients=self.clients();clients['amcl'].service_is_ready.return_value=False
        poll=LifecyclePoll(clients);now=[0.];messages=[]
        def spin(seconds):now[0]+=seconds
        self.assertFalse(wait_active(poll,.31,spin,clock=lambda:now[0],ok=lambda:True,report=messages.append))
        self.assertAlmostEqual(now[0],.31)
        self.assertIn('amcl',messages[-1])
        self.assertIn('timed out',messages[-1])

    def test_active_wait_finishes_before_deadline_and_ros_stop_exits(self):
        poll=LifecyclePoll(self.clients());now=[0.]
        def spin(seconds):now[0]+=seconds
        self.assertTrue(wait_active(poll,30.,spin,clock=lambda:now[0],ok=lambda:True,report=lambda _:None))
        self.assertLess(now[0],.2)
        self.assertFalse(wait_active(poll,30.,spin,clock=lambda:now[0],ok=lambda:False,report=lambda _:None))

    def test_navigation_wait_requires_all_selected_servers_active(self):
        names=('controller_server','planner_server','bt_navigator')
        clients={name:Mock(service_is_ready=Mock(return_value=True),
            call_async=Mock(return_value=future())) for name in names}
        clients['bt_navigator'].call_async.side_effect=[future(state=2),future()]
        poll=LifecyclePoll(clients);now=[0.]
        def spin(seconds):now[0]+=seconds
        messages=[]
        self.assertTrue(wait_active(poll,2.,spin,clock=lambda:now[0],ok=lambda:True,report=messages.append))
        self.assertGreaterEqual(now[0],.25)
        self.assertEqual(set(poll.states),set(names))
        self.assertIn('bt_navigator',messages[-1])
        self.assertTrue(readiness(poll.states,now[0],names=names)[0])


if __name__=='__main__':
    unittest.main()
