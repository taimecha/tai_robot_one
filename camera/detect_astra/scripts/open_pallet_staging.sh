#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$project_dir/ros2_ws/install/local_setup.bash"
exec "${PALLET_PYTHON:-$project_dir/.venv/bin/python}" "$project_dir/scripts/stage_pallet.py" "$@"
