# Ghi chú ca lấy pallet xanh, tối 03/11/2026

## Lần chạy 20:42

Log: `data/diagnostics/pallet_cycle_20261103_204212/`. AMCL đã đạt, Nav2 đã tới goal nhận diện đang lưu `(2.940598741127163, 1.241054822778418, -1.5836497423593432)`. Camera đo tâm lỗ lệch 10,10 cm về bên phải xe, góc cần bù -1,38°. Đường căn ngoài pallet tới checkpoint được chấp nhận.

Chu trình dừng khi quay ở đầu đường căn: tâm xe mới trôi 2,61 mm, nhưng quãng trôi dự báo gồm phanh là 5,30 mm, vượt giới hạn 5 mm. Log odometry trước đó có tốc độ dọc đổi dấu liên tục khi lệnh bù đổi dấu. Càng chưa vào pallet; chưa nâng.

## Thay đổi chương trình

Trong `scripts/pallet_checkpoint.py`, lệnh bù tâm khi quay dùng tốc độ đã lọc 0,2 giây và giới hạn độ đổi lệnh 20 mm/s². Khi quãng trôi dự báo vượt 4 mm, bộ điều khiển ngừng quay, chờ xe đứng yên rồi đưa tâm về trong 1,5 mm tính từ **mốc odometry ban đầu của lượt quay** trước khi quay tiếp. Vẫn dừng nếu tâm trôi thực vượt 5 mm, tốc độ dọc vượt 15 mm/s, không phục hồi trong 6 giây hoặc hãm quá ba lần mỗi lượt quay. Bộ điều khiển tiếp tục kiểm tra quãng phanh, hình học thân/càng và laser. Đoạn đưa càng cuối vẫn giới hạn góc 0,5° và lệch đường 5 mm. Chỉ nâng sau xác nhận trực tiếp `nang`.

## Lần chạy kế tiếp (04/11/2026)

Chỉ chạy khi pallet xanh đã ở đúng vị trí muốn lấy, càng đã hạ và đường tới pallet trống. Chạy trên robot từ thư mục dự án:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_cycle.sh run \
  --auto-localize \
  --revalidate-recognition \
  --pickup-only \
  --usable-depth-cm 35.5
```

Quan sát xe và giữ khả năng dừng khẩn cấp. Nếu chương trình dừng, gửi nguyên dòng lỗi cùng thư mục log mới; giữ nguyên giới hạn an toàn, không tăng 5 mm để vượt lỗi. Điểm lấy hàng bản đồ cũ không được dùng trong chu trình này. Lệnh trên cho xe tự định vị, về goal nhận diện, nhận diện, căn, đo lại ở checkpoint, rồi đưa càng theo odometry; việc nâng chờ người vận hành xác nhận.

Kiểm thử thuật toán: **73 ca đạt**, gồm hình học ca 20:42, phản ứng đế xe có trễ, phanh sớm khi dự báo 5,30 mm, bù về mốc tâm cũ, dừng khi tốc độ trôi vượt ngưỡng và dừng theo lệnh người vận hành. Log ở `data/diagnostics/checkpoint_control_fix_20261103_204212/unit.log`.

Kiểm thử ROS 2 với đế xe, cảm biến và hệ nâng giả lập: **2 ca đạt**. Ca thứ nhất dùng đúng hình học ca 20:42, xác nhận phiên nhận diện mới ở checkpoint, đưa càng và nâng sau lệnh xác nhận. Ca thứ hai xác nhận dừng trước khi đưa càng khi đế xe không đáp ứng lệnh bù tâm. Log ở `data/diagnostics/checkpoint_control_fix_20261103_204212/ros_sim.log`.

Chưa có bằng chứng xe thật đã vượt qua điểm dừng 20:42 hoặc đưa càng thành công với bản sửa này.
