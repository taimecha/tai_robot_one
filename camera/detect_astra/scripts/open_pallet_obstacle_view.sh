#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$project_dir/../../ros2_ws/install/local_setup.bash"
source "$project_dir/data/alignment_robot_ws/install/local_setup.bash"
source "$project_dir/ros2_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/../../ros2_ws/config/camera_fastdds.xml"
exec ros2 launch "$project_dir/launch/pallet_obstacle_view.launch.py"
