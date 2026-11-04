"""Stop an operator child if its cycle supervisor is killed (Linux/Pi)."""
import ctypes
import os
import signal


def arm_cycle_parent():
    expected=os.environ.get('PALLET_CYCLE_PARENT_PID')
    if expected is None:return
    expected=int(expected)
    if expected<=1 or os.getppid()!=expected:raise RuntimeError('Cycle supervisor no longer exists')
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(1,signal.SIGINT,0,0,0)!=0:raise OSError(ctypes.get_errno(),'prctl parent-death signal')
    if os.getppid()!=expected:raise RuntimeError('Cycle supervisor exited during startup')
