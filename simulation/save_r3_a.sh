#!/usr/bin/env bash
# Connect to the running localization session; do not restart sensors/AMCL.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
source /opt/ros/jazzy/setup.bash
source camera/detect_astra/ros2_ws/install/setup.bash
source camera/detect_astra/data/alignment_robot_ws/install/setup.bash
map_file=$(python3 - "$root" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);session=root/'simulation/output/r3_live/localization_session.json'
if not session.is_file():
    raise SystemExit('Chạy run_r3_localization.sh trước khi lưu A.')
state=json.loads(session.read_text());proc=Path(f"/proc/{int(state['pid'])}/cmdline")
if not proc.exists() or b'--localization-only' not in proc.read_bytes().split(b'\0'):
    raise SystemExit('Phiên định vị đã dừng; khởi động lại trước khi lưu A.')
print(state['map_file'])
PY
)
exec python3 simulation/r3_recognition_goal.py save \
  --map "$map_file" \
  --output "$root/simulation/r3/recognition_A.json" "$@"
