"""Stop waits for real process cleanup and inherited flock release; no hardware."""
import fcntl
import json
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from r3_stop_localization import lock_available, process_identity, stop_session


class StopLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.folder=self.root/'simulation/output/r3_live'
        self.folder.mkdir(parents=True)
        self.lock=self.folder/'.trial.lock'
        self.session=self.folder/'localization_session.json'
        self.script=self.root/'simulation/run_r3_live_trial.sh'
        self.processes=[]

    def tearDown(self):
        for process in self.processes:
            try:
                os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
            if process.poll() is None:
                process.kill()
            process.wait(timeout=3)
            if process.stdin is not None:
                process.stdin.close()
            if process.stderr is not None:
                process.stderr.close()
        self.temp.cleanup()

    def start_owner(self, cleanup, mode='--localization-only'):
        self.script.write_text('''#!/usr/bin/env bash
set -e
exec 9>"$LOCK"
flock -x 9
trap '''+"'"+cleanup+"'"+''' TERM INT
touch "$READY"
while true; do sleep .05 & wait $!; done
''')
        ready=self.folder/'ready'
        process=subprocess.Popen(['bash',str(self.script),mode],
            env={**os.environ,'LOCK':str(self.lock),'READY':str(ready)},
            stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,start_new_session=True)
        self.processes.append(process)
        deadline=time.monotonic()+3
        while not ready.exists() and time.monotonic()<deadline:
            if process.poll() is not None:
                self.fail(process.stderr.read().decode())
            time.sleep(.01)
        self.assertTrue(ready.exists(),'fake owner did not acquire its lock')
        identity=process_identity(process.pid)
        self.assertIsNotNone(identity)
        self.session.write_text(json.dumps(dict(pid=process.pid,process_start_ticks=identity[1])))
        self.assertFalse(lock_available(self.lock))
        return process

    def test_returns_only_after_owner_cleanup_finishes_and_lock_is_available(self):
        process=self.start_owner('sleep .3; exit 0')
        started=time.monotonic()
        result=stop_session(self.root,timeout=3.,poll_interval=.01)
        elapsed=time.monotonic()-started
        self.assertGreaterEqual(elapsed,.27)
        self.assertIsNone(process_identity(process.pid))
        self.assertTrue(lock_available(self.lock))
        self.assertIsInstance(result,str)
        process.wait(timeout=1)

    def test_owner_exit_is_insufficient_while_child_inherits_trial_lock(self):
        process=self.start_owner('sleep .6 & exit 0')
        started=time.monotonic()
        stop_session(self.root,timeout=3.,poll_interval=.01)
        self.assertGreaterEqual(time.monotonic()-started,.55)
        self.assertIsNone(process_identity(process.pid))
        self.assertTrue(lock_available(self.lock))
        process.wait(timeout=1)

    def test_absent_session_and_already_stopped_session_are_idempotent(self):
        self.assertIsInstance(stop_session(self.root,timeout=.2),str)
        process=subprocess.Popen(['/bin/true'],start_new_session=True)
        self.processes.append(process)
        process.wait(timeout=1)
        self.session.write_text(json.dumps(dict(pid=process.pid)))
        self.lock.touch()
        for _ in range(2):
            self.assertIsInstance(stop_session(self.root,timeout=.2),str)
            self.assertTrue(lock_available(self.lock))

    def test_unrelated_pid_is_never_signalled(self):
        process=subprocess.Popen(['/bin/sleep','30'],start_new_session=True)
        self.processes.append(process)
        self.session.write_text(json.dumps(dict(pid=process.pid)))
        with self.assertRaisesRegex(RuntimeError,'PID'):
            stop_session(self.root,timeout=.2)
        self.assertIsNone(process.poll())

    def test_reused_owner_pid_start_time_is_never_signalled(self):
        process=self.start_owner('exit 0')
        identity=process_identity(process.pid)
        self.session.write_text(json.dumps(dict(pid=process.pid,process_start_ticks=identity[1]+1)))
        with self.assertRaisesRegex(RuntimeError,'PID'):
            stop_session(self.root,timeout=.2)
        self.assertIsNone(process.poll())
        self.assertFalse(lock_available(self.lock))

    def test_busy_lock_without_localization_owner_has_bounded_timeout(self):
        with self.lock.open('wb') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            started=time.monotonic()
            with self.assertRaisesRegex(RuntimeError,'khóa R3'):
                stop_session(self.root,timeout=.15,poll_interval=.01)
            self.assertGreaterEqual(time.monotonic()-started,.13)
            self.assertLess(time.monotonic()-started,1.)
            self.assertFalse(lock_available(self.lock))

    def test_recorded_navigation_and_check_sessions_can_be_stopped(self):
        for mode in ('--nav-to-a','--nav-check-only','--recognize-here'):
            with self.subTest(mode=mode):
                ready=self.folder/'ready'
                ready.unlink(missing_ok=True)
                process=self.start_owner('sleep .1; exit 0',mode=mode)
                self.assertIsInstance(stop_session(self.root,timeout=3.,poll_interval=.01),str)
                process.wait(timeout=1)
                self.assertIsNone(process_identity(process.pid))
                self.assertTrue(lock_available(self.lock))

    def test_other_mode_of_same_script_is_not_a_localization_session(self):
        process=self.start_owner('exit 0',mode='--unrecognized-mode')
        with self.assertRaisesRegex(RuntimeError,'PID'):
            stop_session(self.root,timeout=.2)
        self.assertIsNone(process.poll())

    def start_foreground_prompt(self,private_group):
        child=self.folder/'foreground.py'
        ready=self.folder/'foreground_ready'
        child_cleanup=self.folder/'foreground_cleanup'
        owner_cleanup=self.folder/'owner_cleanup'
        child.write_text('''import os
from pathlib import Path
try:
    Path(os.environ['READY']).write_text(str(os.getpid()))
    input('Waiting for motion approval: ')
finally:
    Path(os.environ['CHILD_CLEANUP']).touch()
''')
        self.script.write_text('''#!/usr/bin/env bash
set -e
exec 9>"$LOCK"
flock -x 9
trap 'touch "$OWNER_CLEANUP"; exit 0' TERM INT
"$PYTHON" "$FOREGROUND"
''')
        process=subprocess.Popen(['bash',str(self.script),'--nav-to-a'],stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,start_new_session=private_group,
            env={**os.environ,'LOCK':str(self.lock),'READY':str(ready),
                 'CHILD_CLEANUP':str(child_cleanup),'OWNER_CLEANUP':str(owner_cleanup),
                 'PYTHON':sys.executable,'FOREGROUND':str(child)})
        self.processes.append(process)
        deadline=time.monotonic()+3.
        while not ready.exists() and time.monotonic()<deadline:
            if process.poll() is not None:
                self.fail(process.stderr.read().decode())
            time.sleep(.01)
        self.assertTrue(ready.exists(),'foreground prompt did not start')
        identity=process_identity(process.pid)
        self.session.write_text(json.dumps(dict(pid=process.pid,process_start_ticks=identity[1])))
        return process,int(ready.read_text()),child_cleanup,owner_cleanup

    def assert_foreground_prompt_stops(self,private_group):
        process,child,child_cleanup,owner_cleanup=self.start_foreground_prompt(private_group)
        child_identity=process_identity(child)
        self.assertFalse(lock_available(self.lock))
        self.assertEqual(os.getpgid(process.pid)==process.pid,private_group)
        started=time.monotonic()
        try:
            result=stop_session(self.root,timeout=2.,poll_interval=.01)
            self.assertIsInstance(result,str)
            self.assertLess(time.monotonic()-started,1.5)
            self.assertTrue(child_cleanup.exists(),'Python finally was skipped')
            self.assertTrue(owner_cleanup.exists(),'Bash cleanup was skipped')
            self.assertIsNone(process_identity(process.pid))
            self.assertIsNone(process_identity(child))
            self.assertTrue(lock_available(self.lock))
        finally:
            # Clean a failed regression without ever signalling the test group.
            if process_identity(child)==child_identity:
                try:
                    os.kill(child,signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def test_private_owner_group_interrupts_foreground_python_prompt(self):
        self.assert_foreground_prompt_stops(private_group=True)

    def test_shared_terminal_group_stops_only_owner_and_verified_child(self):
        self.assert_foreground_prompt_stops(private_group=False)


if __name__=='__main__':
    unittest.main()
