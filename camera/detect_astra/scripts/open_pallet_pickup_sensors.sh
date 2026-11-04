#!/usr/bin/env bash
# Sensors only. Start the separate cycle terminal to authorize motion.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo 'Usage: bash scripts/open_pallet_pickup_sensors.sh [ROS launch arguments]'
  echo 'Mở phần cứng, EKF/IMU, laser, RGB-D và map/AMCL; không chạy xe hoặc HOME càng.'
  echo 'Giữ terminal mở. Chạy open_pallet_cycle.sh ở terminal khác khi pallet đã sẵn sàng.'
  exit 0
fi
# Do not compete with an existing base/lidar or an earlier alignment process.
python3 - <<'PY'
from pathlib import Path
import sys
names={'align_pallet.py','stage_pallet.py','handle_pallet.py','planner_server','controller_server','bt_navigator','ros2_control_node','sllidar_node','cmd_vel_stamper','ekf_node',
       'collision_monitor','amcl','map_server','pallet_localization_scan.py','pallet_amcl_health.py','esp32_bridge.py','esp32_imu_bridge','astra_camera_node','publish_rgb_calibrated_cloud.py'}
found=[]
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit():continue
 try:
  argv=entry.joinpath('cmdline').read_bytes().split(b'\0')
  if any(Path(arg.decode()).name in names for arg in argv if arg):
   found.append(entry.name+': '+b' '.join(argv).decode())
 except (OSError,UnicodeError):pass
if found:
 print('Đang có chương trình điều khiển/bộ căn chạy. Ctrl+C terminal tương ứng trước khi mở bộ chạy thật:',file=sys.stderr)
 for item in found:print(item,file=sys.stderr)
 sys.exit(1)
PY
for device in /dev/tai_drive /dev/tai_imu /dev/tai_lidar; do
  if [[ ! -r "$device" || ! -w "$device" ]]; then
    echo "Thiếu thiết bị hoặc quyền đọc/ghi: $device" >&2
    exit 1
  fi
done
bash "$project_dir/scripts/build_pallet_alignment_robot.sh"
# Source local_setup to avoid obsolete underlays recorded in old workspaces.
source "$project_dir/../../ros2_ws/install/local_setup.bash"
source "$project_dir/data/alignment_robot_ws/install/local_setup.bash"
source "$project_dir/ros2_ws/install/local_setup.bash"
export PALLET_PYTHON=${PALLET_PYTHON:-$project_dir/.venv/bin/python}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/../../ros2_ws/config/camera_fastdds.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
exec ros2 launch "$project_dir/launch/pallet_pickup_sensors.launch.py" "$@"
