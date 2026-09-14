# Bản đồ firmware chính

Chỉ hai thư mục dưới đây được dùng trên xe:

| Bo | Chức năng | Project PlatformIO | File chính | Cổng Pi ổn định |
| --- | --- | --- | --- | --- |
| ESP32 số 1 | PID và encoder 4 bánh | `amr_firmware` | `amr_firmware/src/main.cpp` | `/dev/tai_drive` |
| ESP32 số 2 | BNO055 và càng TB6600 | `bno055_imu_monitor` | `bno055_imu_monitor/src/main.cpp` | `/dev/tai_imu` |

Cả hai firmware chỉ giao tiếp bằng USB Serial; Wi-Fi và Bluetooth đều tắt.
