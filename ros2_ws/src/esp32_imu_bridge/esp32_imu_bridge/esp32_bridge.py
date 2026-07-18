import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ESP32ImuBridge(Node):
    def __init__(self):
        super().__init__('esp32_imu_bridge_node')
        
        # Giữ nguyên cấu hình QoS Best Effort chuyên dụng chống Jitter
        imu_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,           
            depth=1,                                   
            durability=DurabilityPolicy.VOLATILE       
        )
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', imu_qos_profile)
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info('HỆ THỐNG SẴN SÀNG: Đang lắng nghe luồng dữ liệu từ ESP32...')
        except Exception as e:
            self.get_logger().error(f"Lỗi cổng Serial, vui lòng kiểm tra cáp cắm: {e}")
            return
        
        # Vòng lặp quét cổng USB liên tục
        self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Bắt và in các chuỗi thông báo "STATUS:" (đếm ngược 5 giây) từ setup() của ESP32
                if line.startswith("STATUS:"):
                    self.get_logger().info(line)
                    return 
                
                data = line.split(',')
                
                if len(data) >= 7:
                    # Đọc 7 thông số đã được ESP32 tính toán ma trận bù góc xoay
                    qx = float(data[0])  # Quaternion X
                    qy = float(data[1])  # Quaternion Y
                    qz = float(data[2])  # Quaternion Z
                    qw = float(data[3])  # Quaternion W
                    
                    ax = float(data[4])  # Gia tốc tuyến tính X
                    ay = float(data[5])  # Gia tốc tuyến tính Y
                    az = float(data[6])  # Gia tốc tuyến tính Z

                    # 🎯 DÒNG LOG ĐỂ BẠN ĐỐI CHIẾU TRỰC TIẾP TRÊN TERMINAL
                    # Format hiển thị rõ ràng dấu + - và cố định 3 chữ số thập phân cho dễ nhìn
                    self.get_logger().info(
                        f"IMU_CHECK -> Quaternion[X:{qx:+.3f}, Y:{qy:+.3f}, Z:{qz:+.3f}, W:{qw:+.3f}] | Accel_Z: {az:.2f}"
                    )

                    # Đóng gói khung thông điệp ROS 2
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'

                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw

                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az

                    self.imu_pub.publish(imu_msg)
                    
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()