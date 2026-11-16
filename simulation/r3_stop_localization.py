#!/usr/bin/env python3
"""Stop the recorded localization owner and await its cleanup and lock release."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import time


def lock_available(path):
    try:
        stream=Path(path).open('rb')
    except FileNotFoundError:
        return True
    with stream:
        try:
            fcntl.flock(stream,fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        return True


def process_identity(pid):
    try:
        args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        # Everything before the closing ')' is pid/comm, which may contain
        # spaces; field 22 is starttime, index 19 after that delimiter.
        stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        if not any(args) or stat[0]=='Z':
            return None
        return args,int(stat[19])
    except (FileNotFoundError,ProcessLookupError):
        return None


def process_relationship(pid):
    """Read the kernel parent/group, never infer ownership from a PID alone."""
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return int(fields[1]),int(fields[2])
    except (FileNotFoundError,ProcessLookupError):
        return None


def interrupt_owner(pid,owner):
    """Unblock a foreground prompt while retaining the owner's cleanup traps.

    Bash defers a TERM trap while waiting for a foreground Python process.
    INT reaches Python's finally blocks as well as Bash's cleanup trap. Only
    a verified private group may receive a group signal; a shared terminal
    group is never interrupted.
    """
    if process_identity(pid)!=owner:
        return
    try:
        pgid=os.getpgid(pid)
    except ProcessLookupError:
        return
    if pgid==pid and pgid!=os.getpgrp():
        if process_identity(pid)==owner:
            try:
                os.killpg(pgid,signal.SIGINT)
            except ProcessLookupError:
                pass
        return

    # A shell may have launched the trial inside a larger foreground group.
    # Its direct children in that same group are foreground work; separately
    # sessioned ROS launch groups remain the owner's cleanup responsibility.
    children=[]
    try:
        entries=Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
    except (FileNotFoundError,ProcessLookupError):
        entries=[]
    for child in map(int,entries):
        if child==os.getpid() or process_relationship(child)!=(pid,pgid):
            continue
        identity=process_identity(child)
        if identity is not None:
            children.append((child,identity))
    if process_identity(pid)!=owner:
        return
    try:
        os.kill(pid,signal.SIGTERM)
    except ProcessLookupError:
        return
    for child,identity in children:
        if (process_identity(pid)==owner and process_identity(child)==identity
                and process_relationship(child)==(pid,pgid)):
            try:
                os.kill(child,signal.SIGINT)
            except ProcessLookupError:
                pass


def stop_session(root,timeout=90.,poll_interval=.05):
    if not math.isfinite(timeout) or timeout<=0 or not math.isfinite(poll_interval) or poll_interval<=0:
        raise ValueError('timeout and poll_interval must be finite and positive')
    root=Path(root).resolve()
    folder=root/'simulation/output/r3_live'
    session=folder/'localization_session.json'
    lock=folder/'.trial.lock'
    owner=None;pid=None
    if session.is_file():
        try:
            state=json.loads(session.read_text())
            pid=state['pid']
            if isinstance(pid,bool) or not isinstance(pid,int) or pid<=1:
                raise ValueError('invalid process identity')
        except (KeyError,ValueError,TypeError) as exc:
            raise RuntimeError('Tệp phiên định vị không hợp lệ; không gửi tín hiệu.') from exc
        identity=process_identity(pid)
        if identity is not None:
            args,started=identity
            expected=str(root/'simulation/run_r3_live_trial.sh').encode()
            session_modes=(b'--localization-only',b'--nav-to-a',b'--nav-check-only',b'--recognize-here')
            if (expected not in args or not any(mode in args for mode in session_modes)
                    or ('process_start_ticks' in state and state['process_start_ticks']!=started)):
                raise RuntimeError('PID không còn thuộc phiên định vị; không gửi tín hiệu.')
            owner=identity
            interrupt_owner(pid,owner)
            print('Đang đóng phiên định vị, chờ các tiến trình con dừng và nhả khóa...',flush=True)
    deadline=time.monotonic()+timeout
    while True:
        same_owner=owner is not None and process_identity(pid)==owner
        if not same_owner and lock_available(lock):
            return 'Đã đóng phiên định vị và nhả khóa; có thể chạy lượt mới.'
        if time.monotonic()>=deadline:
            raise RuntimeError(f'Chưa đóng hoàn toàn sau {timeout:g} giây; khóa R3 vẫn có thể đang được giữ. '
                               'Chưa được chạy lượt mới; kiểm tra tiến trình của phiên cũ.')
        time.sleep(min(poll_interval,max(0.,deadline-time.monotonic())))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--timeout',type=float,default=90.)
    args=parser.parse_args()
    try:
        print(stop_session(args.root,args.timeout),flush=True)
        return 0
    except (RuntimeError,ValueError,OSError) as exc:
        print(str(exc),flush=True)
        return 2


if __name__=='__main__':
    raise SystemExit(main())
