import smbus2  # type: ignore
import time
import os

bus = smbus2.SMBus(1)
ADDRESS = 0x2D

def read_word_little_endian(reg):
    """Đọc dữ liệu 16-bit bảo vệ lỗi, tự động trả về None nếu thanh ghi không tồn tại"""
    try:
        low_byte = bus.read_byte_data(ADDRESS, reg)
        high_byte = bus.read_byte_data(ADDRESS, reg + 1)
        return (high_byte << 8) | low_byte
    except Exception:
        return None

def read_ups_data():
    # ĐỌC CỤM THANH GHI CHẴN AN TOÀN CỦA WAVESHARE
    raw_v20 = read_word_little_endian(0x20)  # Thanh ghi chứa dữ liệu áp thô
    raw_curr = read_word_little_endian(0x22) # Thanh ghi dòng điện (mA)
    raw_pct = read_word_little_endian(0x24)  # Thanh ghi phần trăm (%)
    raw_v26 = read_word_little_endian(0x26)  # Thanh ghi dung lượng mAh thô
    
    # Bộ lọc bảo vệ: Chỉ báo lỗi mất kết nối nếu không đọc được Dòng và %
    if raw_curr is None or raw_pct is None:
        return None, None, None, None, False, None, None

    # THUẬT TOÁN TÍNH TOÁN ĐIỆN ÁP TỰ ĐỘNG ĐO ĐẠC
    if raw_v20 is not None and raw_v20 > 0:
        if raw_v20 > 5000:
            # Nếu thanh ghi trả về mV tổng (Ví dụ: 15800 mV)
            voltage = raw_v20 / 1000.0
        else:
            # Nếu thanh ghi trả về định dạng áp định danh tỉ lệ
            voltage = (raw_v20 * 7.2) / 1000.0
    else:
        # Giá trị dự phòng nếu thanh ghi 0x20 trống
        voltage = 15.8

    # Xử lý dòng điện có dấu Signed 16-bit (mA)
    current_ma = raw_curr
    if current_ma & 0x8000:
        current_ma = current_ma - 0x10000
    
    is_charging = True if current_ma > 10 else False
    power_w = voltage * (abs(current_ma) / 1000.0)
    
    # Chuẩn hóa dung lượng thực tế (%) từ IC thanh ghi 0x24
    percent = raw_pct / 10.0 if raw_pct > 100 else raw_pct
    percent = min(max(percent, 0.0), 100.0)
    
    return voltage, current_ma, power_w, percent, is_charging, raw_v20, raw_v26

try:
    while True:
        os.system('clear')
        v, c, p, pct, charging, r20, r26 = read_ups_data()
        
        if c is not None:
            v_cell = v / 4.0  # Cấu trúc khối 4 cell pin EVE 21700
            
            if charging:
                lbl_dong = "Dòng điện SẠC "
                lbl_cong_suat = "Công suất SẠC"
                lbl_trang_thai = "MẠCH UPS ĐANG SẠC PIN (CẮM NGUỒN NGOÀI)"
            else:
                lbl_dong = "Dòng điện XẢ  "
                lbl_cong_suat = "Công suất TẢI"
                lbl_trang_thai = "XE ĐANG CHẠY PIN (VẬN HÀNH TỰ HÀNH)"
                
            print("==================================================")
            print("    HỆ THỐNG QUẢN LÝ NGUỒN THÔNG MINH AGV - CTU   ")
            print("==================================================")
            print(f" Thời gian thực  : {time.strftime('%H:%M:%S')}")
            print(f" Điện áp Tổng 4S : {v:.3f} V")
            print(f" Điện áp mỗi Cell: {v_cell:.3f} V")
            print(f" Dung lượng pin  : {pct:.1f} %")
            print("-" * 50)
            print(f" {lbl_dong}   : {c} mA")
            print(f" {lbl_cong_suat}  : {p:.2f} W")
            print("-" * 50)
            print("   [HỘ TRỢ THEO DÕI LOG ĐỂ HIỆU CHUẨN ĐIỆN ÁP]   ")
            print(f" Giá trị thô thanh ghi 0x20: {r20}")
            print(f" Giá trị thô thanh ghi 0x26: {r26}")
            print("==================================================")
            print(f" Trạng thái: {lbl_trang_thai}")
            print("==================================================")
            print(" Nhấn Ctrl + C để thoát màn hình giám sát.")
        else:
            print("Lỗi: Không kết nối hoặc chưa đồng bộ được với mạch UPS!")
            
        time.sleep(1)

except KeyboardInterrupt:
    print("\n Đã đóng trình giám sát nguồn.")