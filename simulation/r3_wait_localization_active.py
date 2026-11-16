#!/usr/bin/env python3
"""Read-only, bounded wait for map_server and AMCL lifecycle activation."""
import argparse
import math
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


NAMES=('map_server','amcl')
REQUEST_TIMEOUT_S=2.


def readiness(states, now, max_age_s=2.5, *, names=NAMES):
    """Require a fresh successful ACTIVE response from both named services."""
    for name in names:
        state=states.get(name,{})
        if state.get('error'):
            return False,f'{name}: {state["error"]}'
        stamp=state.get('stamp')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not 0<=now-stamp<=max_age_s:
            return False,f'{name}: missing/stale lifecycle response'
        if state.get('id')!=3:
            return False,f'{name}: lifecycle state {state.get("id")} is not ACTIVE (3)'
    return True,' and '.join(names)+' lifecycle ACTIVE'


class LifecyclePoll:
    def __init__(self, clients):
        self.clients=clients
        self.states={}
        self.pending={}
        self.next_poll={name:0. for name in clients}

    def tick(self, wall):
        for name in self.clients:
            client=self.clients[name]
            try:
                if name in self.pending:
                    future,sent=self.pending[name]
                    if future.done():
                        del self.pending[name]
                        response=future.result()
                        self.states[name]=dict(id=response.current_state.id,stamp=wall)
                        self.next_poll[name]=wall+.25
                    elif wall-sent>=REQUEST_TIMEOUT_S:
                        del self.pending[name]
                        client.remove_pending_request(future)
                        self.states[name]=dict(error='get_state request timed out')
                        self.next_poll[name]=wall+.25
                    continue
                if wall<self.next_poll[name]:
                    continue
                self.next_poll[name]=wall+.25
                if not client.service_is_ready():
                    self.states[name]=dict(error='get_state service unavailable')
                    continue
                self.pending[name]=(client.call_async(GetState.Request()),wall)
            except Exception as exc:
                self.pending.pop(name,None)
                self.next_poll[name]=wall+.25
                self.states[name]=dict(error='get_state failed: '+str(exc))


def wait_active(poll, timeout, spin, *, clock=time.monotonic, ok=rclpy.ok, report=print):
    deadline=clock()+timeout
    last_reason=None
    while ok() and clock()<deadline:
        wall=clock()
        poll.tick(wall)
        ready,reason=readiness(poll.states,wall,names=tuple(poll.clients))
        if ready:
            report(reason)
            return True
        if reason!=last_reason:
            report('Waiting for localization activation: '+reason)
            last_reason=reason
        spin(min(.05,max(0.,deadline-clock())))
    report('Localization activation timed out/stopped: '+str(last_reason))
    return False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout',type=float,default=30.)
    parser.add_argument('--nodes',nargs='+',default=list(NAMES),
                        help='Lifecycle node names whose ACTIVE state is required')
    args=parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<=0:
        parser.error('--timeout must be finite and positive')
    names=tuple(name.strip('/') for name in args.nodes)
    if not names or any(not name or '//' in name for name in names) or len(set(names))!=len(names):
        parser.error('--nodes requires distinct nonempty node names')
    rclpy.init()
    node=Node('r3_wait_localization_active')
    try:
        poll=LifecyclePoll({name:node.create_client(GetState,f'/{name}/get_state') for name in names})
        active=wait_active(poll,args.timeout,lambda seconds:rclpy.spin_once(node,timeout_sec=seconds),
                           report=lambda message:print(message,flush=True))
        return 0 if active else 2
    except KeyboardInterrupt:
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
