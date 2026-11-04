# Giao pallet 1 và hạ hàng bằng lệnh riêng

Lệnh dùng Nav2 thật đang active, định vị `map → base_footprint` mới, LiDAR và bridge IMU/càng. Không mở cùng bộ căn/đưa càng đang giữ lệnh xe bằng 0. Càng phải đang mang pallet, HOME đã xác nhận từ lượt trước; chuyển sang Nav2 không tự HOME.

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_delivery.sh go
# Đợi ARRIVED: xe tới goal, đứng yên rồi mới chạy:
bash scripts/open_pallet_delivery.sh lower
```

Goal lấy từ `ros2_ws/config/pallet_goals.yaml`, pallet_1.dropoff: X 0,2009506 m, Y −0,4480836 m, yaw −180°. Không sửa tọa độ đã lưu. `go` dùng NavigateToPose với BT/costmap/Collision Monitor đã chỉnh trên xe thật. Trước gửi goal, đặt footprint chở pallet thành hình chữ nhật X −0,33…0,80 m, Y ±0,25 m trong cả hai costmap, tốc độ mong muốn 0,12 m/s và quay 0,20 rad/s. Footprint này bao kích thước pallet xanh đã quan sát khoảng rộng 34 cm và sâu 35 cm; pallet/load lớn hơn cần footprint tương ứng trước khi chở.

`lower` chỉ gửi SET về hành trình 0 sau khi ở đúng goal (trong 8 cm, 5°), Nav2 không chạy và xe đứng yên liên tục 0,8 giây. Kiểm tra LiDAR/odom/phản hồi càng mới, HOME/fault và giữ xe đứng yên trong lúc hạ. Không gửi HOME, không tự lùi rút càng. LOWERED xác nhận hành trình càng về 0 và MCU đứng yên, không phải cảm biến tiếp xúc pallet/sàn.

Giữ terminal mở; Ctrl+C gửi hủy Nav2/dừng xe/càng. Từ terminal khác:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_delivery.sh stop
```

Ngày 2026-11-02 đã chuyển khỏi stack căn, giữ càng 8 cm, định vị lại bằng đối chiếu LiDAR/bản đồ rồi AMCL đứng yên: khoảng (2,958; −0,130), yaw −88,26°. Planner GridBased trả đường 56 pose tới dropoff với footprint chở pallet. Chưa gửi goal vận chuyển; người vận hành yêu cầu tự chạy hai lệnh trên. Pose AMCL lưu cũ khác vị trí thực nên đã tắt khôi phục pose cũ trong lần mở này; không gán goal pickup làm vị trí thật của xe.

### Điều tra lượt hủy 208 và nguồn sàn giả

Lượt do người vận hành chạy ngày 2026-11-02 đã quay xe tới khoảng −147°, sau đó Nav2 trả NO_VALID_PATH 208. BT chặn footprint trên đường và phép xoay cuối; feedback `distance_remaining=0` trong recovery không phải xác nhận đã tới. CLI nay in riêng khoảng cách hình học tới goal và báo rõ lỗi 208.

Đã xác nhận các ô lethal gây chặn tại vùng (0,56; −0,72) và (0,91; −0,47) là dấu động, bản đồ PGM ở đó vẫn free. Camera cũ nhận một phần sàn trống cao 6,0–6,6 cm. Hồ sơ bù cao độ mới `astra_pro_nav_floor_2026_11_02.yaml` đã được nạp, giữ nguyên X/Y, cửa sổ vật cản và footprint; global/local costmap được làm mới khi xe đứng yên và không có action active. Càng giữ 8 cm.

Sau khi nạp, vùng sàn kiểm tra trực tiếp bị che bởi người ngồi sát xe và ghế được đưa ra cạnh bàn; ảnh RGB/cloud đồng bộ lệch 20,8 ms xác nhận các điểm mới thuộc vật cản thật. Không tiếp tục sửa mặt phẳng từ vùng đã bị che, không xóa các vật cản thật để ép đường đi. Chưa kiểm chứng lại được đường giao hàng khi lối trống; đang chờ người vận hành dọn lối. Xe không được khởi động lại bằng công cụ sau khi người vận hành yêu cầu tự chạy.

Trong phiên này node cloud được nạp riêng sau khi dừng node cũ để giữ nguyên phần cứng/càng. Log `/tmp/pallet_corrected_cloud_20261102.log`; phải dừng node cloud riêng này cùng stack Nav2 trước khi mở một stack mới để tránh hai nguồn cloud. Lần mở mới qua `nav_real.launch.py` sẽ tự dùng hồ sơ ngày 02-11.

### Sau khi người vận hành dọn lối

Ảnh RGB/cloud ghép lệch 41 ms xác nhận 4.912 điểm trong vùng sàn trống, không điểm nào cao >=6 cm; percentile 99 cao độ 0,65 cm. Xe đứng yên, càng MCU IDLE tại 8 cm, không có action active. Đã làm mới costmap động một lần nữa để bỏ dấu người/ghế đã di chuyển. GridBased và SE2Fallback đều tìm được đường; đường GridBased đi chéo vẫn bị kiểm tra góc cuối từ chối.

Phát lại dữ liệu với đúng plugin `PathFootprintClear` đang cài trong ROS domain 231, chỉ phát costmap/footprint giả và không có publisher vận tốc/càng, đã xác nhận phương án xoay đầu rồi đi thẳng tới đúng dropoff là SUCCESS qua toàn bộ footprint, đoạn xoay đầu và đoạn xoay cuối. Giữ nguyên footprint chở pallet, goal và các ngưỡng bảo vệ. Dữ liệu: `delivery_ready_direct_validation_20261102.json`. Chưa gửi NavigateToPose hoặc lệnh nâng/hạ; người vận hành tự chạy lại `go`, đợi ARRIVED rồi mới `lower`.
