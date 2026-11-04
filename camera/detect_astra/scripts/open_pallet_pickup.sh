#!/usr/bin/env bash
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
source /opt/ros/jazzy/setup.bash
source "$project_dir/ros2_ws/install/local_setup.bash"
exec "${PALLET_PYTHON:-$project_dir/.venv/bin/python}" "$project_dir/scripts/run_pallet_pickup.py" "$@"
