#!/usr/bin/env bash
# Run this on the Ubuntu laptop, beside the downloaded r3_laptop.rviz.
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo 'Usage: bash open_r3_laptop_rviz.sh [path/to/r3_laptop.rviz]'
  echo 'Run on an Ubuntu 24.04 desktop with ROS 2 Jazzy, on the same LAN as the Pi.'
  exit 0
fi
[[ $# -le 1 ]] || { echo 'Chỉ truyền một đường dẫn cấu hình RViz.' >&2; exit 2; }
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# -eq 1 ]]; then
  config=$1
elif [[ -f "$script_dir/r3_laptop.rviz" ]]; then
  config="$script_dir/r3_laptop.rviz"
else
  config="$script_dir/../ros2_ws/config/r3_laptop.rviz"
fi
[[ -r "$config" ]] || { echo "Không đọc được cấu hình RViz: $config" >&2; exit 2; }
[[ -r /opt/ros/jazzy/setup.bash ]] || {
  echo 'Laptop cần ROS 2 Jazzy tại /opt/ros/jazzy.' >&2
  exit 1
}
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

for package in rviz2 nav2_rviz_plugins rmw_fastrtps_cpp; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || {
    echo "Laptop thiếu gói ROS: $package" >&2
    echo 'Cài bằng lệnh: sudo apt update && sudo apt install -y ros-jazzy-rviz2 ros-jazzy-nav2-rviz-plugins ros-jazzy-rmw-fastrtps-cpp' >&2
    exit 1
  }
done
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || {
  echo 'Mở terminal trên màn hình desktop Ubuntu của laptop để chạy RViz.' >&2
  exit 1
}
echo "RViz laptop: domain=$ROS_DOMAIN_ID, Fast DDS, SUBNET; cấu hình $config"
exec rviz2 -d "$config" --ros-args -p use_sim_time:=false
