"""Fresh sensor windows before an operator sends a movement request."""
import math


class SensorWindow:
    def __init__(self,timeout_s,settle_s=5.):
        self.timeout_s=timeout_s;self.settle_s=settle_s
        self.stamp=self.received=self.since=None

    def observe(self,stamp,wall,ros):
        if not all(math.isfinite(x) for x in (stamp,wall,ros)):
            self.since=None;return
        if self.stamp is not None and stamp<=self.stamp:return
        gap=self.received is not None and wall-self.received>self.timeout_s
        capture_gap=self.stamp is not None and stamp-self.stamp>self.timeout_s
        self.stamp,self.received=stamp,wall
        if not -.05<=ros-stamp<=self.timeout_s:
            self.since=None
        elif self.since is None or gap or capture_gap:self.since=wall

    def ready(self,wall,ros):
        fresh=(self.received is not None and 0<=wall-self.received<=self.timeout_s
               and -.05<=ros-self.stamp<=self.timeout_s)
        if not fresh:self.since=None
        return fresh and self.since is not None and wall-self.since>=self.settle_s
