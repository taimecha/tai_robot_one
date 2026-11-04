"""Scheduling changes affect every existing thread and restore their masks."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]


class CpuBudgetTest(unittest.TestCase):
    def test_child_thread_masks_are_restricted_then_restored(self):
        if len(os.sched_getaffinity(0))<4:self.skipTest('needs four CPUs')
        code='''
import os,json,threading
from pallet_cpu_budget import CpuBudget
stop=threading.Event();t=threading.Thread(target=stop.wait);t.start()
tid=t.native_id
before=os.sched_getaffinity(0);worker_before=os.sched_getaffinity(tid)
b=CpuBudget();b.apply_process(os.getpid())
selected=os.sched_getaffinity(0);worker_selected=os.sched_getaffinity(tid)
b.restore();after=os.sched_getaffinity(0);worker_after=os.sched_getaffinity(tid)
stop.set();t.join()
print(json.dumps(dict(before=sorted(before),selected=sorted(selected),after=sorted(after),
    worker_before=sorted(worker_before),worker_selected=sorted(worker_selected),worker_after=sorted(worker_after),
    command=b.command(['ros2','launch','example']))))
'''
        result=subprocess.run([sys.executable,'-c',code],env={**os.environ,'PYTHONPATH':str(ROOT/'scripts')},
                              capture_output=True,text=True,check=True,timeout=5)
        data=json.loads(result.stdout)
        self.assertEqual(data['selected'],data['before'][:-1])
        self.assertEqual(data['worker_selected'],data['selected'])
        self.assertEqual(data['after'],data['before'])
        self.assertEqual(data['worker_after'],data['worker_before'])
        self.assertEqual(data['command'][:2],['taskset','--cpu-list'])


if __name__=='__main__':unittest.main()
