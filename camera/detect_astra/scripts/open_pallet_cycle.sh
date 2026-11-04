#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$project_dir/../../ros2_ws/install/local_setup.bash"
source "$project_dir/data/alignment_robot_ws/install/local_setup.bash"
source "$project_dir/ros2_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/../../ros2_ws/config/camera_fastdds.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
exec "${PALLET_PYTHON:-$project_dir/.venv/bin/python}" "$project_dir/scripts/run_pallet_cycle.py" "$@"
