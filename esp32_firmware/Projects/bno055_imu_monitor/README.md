# BNO055 IMU monitor (bench project)

Project này được tạo từ code web monitor do người dùng cung cấp. Nó giữ giao
diện Wi-Fi ở `http://192.168.5.1`, đồng thời bổ sung frame USB 115200 baud để
`esp32_imu_bridge` có thể phát `sensor_msgs/Imu` đầy đủ hơn.

Bản 2.170 dòng do người dùng gửi được lưu nguyên nội dung (chỉ chuẩn hóa xuống
dòng CRLF thành LF) tại `reference/original_web_monitor.cpp`. File build chính
là `src/main.cpp`: bản này refactor giao diện gọn hơn và thêm gyro/protocol USB.

## Đấu dây

| BNO055 | ESP32 |
| --- | --- |
| VIN | 3V3 (hoặc theo đúng breakout đang dùng) |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Địa chỉ mặc định là `0x28`; đổi `kBnoAddress` thành `0x29` nếu chân ADR của
module yêu cầu. Web AP là `BNO055_IMU_MONITOR`, mật khẩu bench mặc định
`12345678` và phải đổi nếu dùng ngoài bàn thử.

## USB protocol

Ở 25 Hz firmware phát:

```text
IMU,1,sequence,sample_ms,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,sys,gyr,acc,mag
```

Gyro dùng rad/s, acceleration dùng m/s². Các dòng `BOOT,...` và `STATUS:...`
chỉ là diagnostics; bridge bỏ qua chúng an toàn.

## Giới hạn kiến trúc hiện tại

Đây là project monitor độc lập để lưu và thử code IMU, không phải firmware
production của ESP32 càng. Robot đã chốt hai ESP32 và hai cổng USB cho drive +
lift; không nạp project này lên một trong hai board khi chạy robot. Nếu muốn
BNO055 hoạt động trên robot mà không thêm ESP32 thứ ba, bước production đúng là
nối BNO055 vào ESP32 càng và mở rộng protocol lift/backend `ros2_control` để
dùng chung `/dev/tai_lift`.
