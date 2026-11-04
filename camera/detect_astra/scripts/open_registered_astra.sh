#!/usr/bin/env bash
set -eo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
astra_overlay=${ASTRA_OVERLAY:-$project_dir/../../ros2_ws/install}
source "$astra_overlay/setup.bash"
set -u

# Match the validated real-navigation camera transport and stream rates.
# Default Astra launch uses RGB 30 Hz and a 512-KiB DDS SHM segment, which
# cannot hold one uncompressed 640x480 RGB frame (921600 bytes).
camera_transport="$project_dir/../../ros2_ws/config/camera_fastdds.xml"
if [[ ! -r "$camera_transport" ]]; then
  echo "Thiếu cấu hình truyền RGB-D: $camera_transport" >&2
  exit 1
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$camera_transport}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$FASTRTPS_DEFAULT_PROFILES_FILE}"

rgb_calibration_file="$project_dir/config/calibration/astra_pro_rgb_640x480_2026-10-11.yaml"
if [[ ! -r "$rgb_calibration_file" ]]; then
  echo "Không đọc được file hiệu chuẩn RGB: $rgb_calibration_file" >&2
  exit 1
fi

for astra_product_id in 0403 0501; do
  astra_usb_device=$(lsusb -d "2bc5:$astra_product_id" | awk 'NR == 1 { gsub(":", "", $4); printf "/dev/bus/usb/%s/%s", $2, $4 }')
  if [[ -z "$astra_usb_device" ]]; then
    echo "Chưa thấy thiết bị Astra USB 2bc5:$astra_product_id." >&2
    exit 1
  fi
  if [[ ! -w "$astra_usb_device" ]]; then
    echo "Astra chưa có quyền truy cập: $astra_usb_device (2bc5:$astra_product_id)" >&2
    echo "Chạy: sudo chgrp plugdev $astra_usb_device && sudo chmod g+rw $astra_usb_device" >&2
    exit 1
  fi
done

if pgrep -f 'astra_camera_node( |$)' >/dev/null; then
  echo 'Astra camera node đã chạy. Kiểm tra depth_registration trước khi mở thêm node.' >&2
  exit 1
fi

exec ros2 launch astra_camera astra_pro.launch.xml \
  depth_registration:=true \
  enable_color:=true \
  enable_ir:=false \
  color_width:=640 color_height:=480 color_fps:=15 \
  depth_width:=640 depth_height:=480 depth_fps:=30 \
  tf_publish_rate:=0.0 \
  use_uvc_camera:=true \
  enable_point_cloud:="${ASTRA_POINT_CLOUD:-true}" \
  enable_colored_point_cloud:=false \
  color_info_url:="file://$rgb_calibration_file"
