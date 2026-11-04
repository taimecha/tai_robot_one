> Historical workflow; do not use these commands for the current pickup controller.

# Đưa càng vào và nâng pallet sau căn chỉnh

Cập nhật 2026-11-02. Người vận hành tự chạy từng lệnh; mở stack hoặc nhận diện xong không tự đưa càng hay nâng.

`handle_pallet.py` giữ nguyên bộ căn `PalletStagingNode` rồi bổ sung bước cuối. Nav2 dùng cho căn ngang; đoạn đưa càng bám một đường thẳng nông bằng odometry, không gọi lại planner 2D để tìm đường xuyên qua pallet. Dữ liệu cloud/sàn thử nghiệm trong `pallet_insertion_scene.py` không phải điều kiện chạy của chế độ này.

## Mở lại từ đầu trên Pi 5

Dừng stack hardware/Nav2/căn cũ bằng Ctrl+C trước; không mở hai stack điều khiển đồng thời. Mỗi khối dưới chạy trong một terminal riêng, cùng `ROS_DOMAIN_ID` như trước.

Camera, nếu chưa chạy:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
ASTRA_POINT_CLOUD=false bash scripts/open_registered_astra.sh
```

Nhận diện và đo bằng hồ sơ đã hiệu chỉnh:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 \
  --target-color green \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml \
  --heading-floor-reference --start-paused
```

Web:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_web.sh
```

Hardware, Nav2 căn và bộ đưa càng/nâng mới:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_alignment_real.sh
```

Script này build/cài bridge mới có `/lift/stop`, không cần nạp lại firmware ESP32. `use_lift=false` ở ros2_control vẫn đúng: càng do **bridge IMU/lift ESP32 kết hợp** quản lý trên `/dev/tai_imu`, không phải controller effort mô phỏng.

Nếu vừa khởi động ESP32 và càng chưa HOME, khi xe đang đứng yên, càng ở ngoài pallet và chưa mang hàng, gửi HOME một lần trong terminal khác:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /lift/home std_msgs/msg/Empty '{}'
ros2 topic echo /lift/status
```

Đợi `homed: true`, `state: IDLE`, `position` gần 0, `fault: NONE` rồi Ctrl+C lệnh **echo**. Ctrl+C echo chỉ dừng xem trạng thái. Muốn dừng chuyển động HOME dùng `/lift/stop` như ở dưới. Bộ đưa càng không tự HOME khi càng đã trong pallet.

Pallet xanh là pallet 1: `--target-color green` lọc màu trong mặt nạ YOLO để chỉ chọn pallet xanh, kể cả khi pallet thường cũng nằm trong ảnh. Không đánh số theo bên trái/phải hay theo confidence lớn nhất. Nếu thấy hai pallet xanh hợp lệ, vẫn báo mục tiêu chưa duy nhất.

Trên web bấm **Nhận diện lại**, đợi RGB-D ổn định, rồi **Bắt đầu căn xe**. Chỉ chuyển sang lệnh tiếp theo khi báo **ALIGNED / Đã căn đạt** và xe đứng yên.

Hoặc bắt đầu căn bằng terminal sau khi nhận diện đã xác nhận pallet:

```bash
source /opt/ros/jazzy/setup.bash
ros2 service call /pallet/alignment/set_enabled std_srvs/srv/SetBool '{data: true}'
```

Một yêu cầu chờ YOLO dừng tối đa khoảng 1 giây, sau đó chờ tối đa 7 giây cho phép đo mới và cửa sổ RGB-D 5 giây đạt. Xe giữ lệnh 0 trong lúc chờ; ngưỡng ngắt 250 ms, độ mới odometry/RGB-D và các điều kiện Nav2 không đổi. Nếu chưa nhận diện đúng pallet thì từ chối ngay; nếu dữ liệu vẫn chưa đạt khi hết thời gian thì trả lý do và không tự thử lại. Lệnh Dừng hủy cả yêu cầu đang chờ, dữ liệu phục hồi sau đó không tự chạy:

```bash
ros2 service call /pallet/alignment/set_enabled std_srvs/srv/SetBool '{data: false}'
```

Nếu nút bắt đầu bị mờ, xem lý do chờ hiện tại. Sau lỗi odometry thoáng qua, dữ liệu mới không tự khởi động xe; vẫn cần bấm bắt đầu lại khi đủ điều kiện. Web ưu tiên hiển thị điều kiện đang chặn (ví dụ cửa sổ RGB-D chưa đạt) thay cho lỗi odometry cũ. Từ bản sửa 2026-11-02, `open_registered_astra.sh` dùng cùng profile truyền DDS ảnh lớn và RGB 15 Hz/depth 30 Hz như cấu hình camera Nav2 đã tối ưu; cần mở lại terminal camera để áp dụng. Bộ căn chỉ giữ mẫu odometry mới nhất và vẫn kiểm tra tuổi tối đa 200 ms.

## Lệnh người vận hành

Trong terminal lệnh trên Pi:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_pickup.sh preview
```

`preview` chốt thử từ phép đo **mới**, kiểm tra toàn bộ đường với laser và in chiều dài/goal/cao độ. Chưa gửi lệnh xe/càng. Thêm `--json` để xem toàn bộ các điểm đường. Lệnh chạy sẽ chốt lại bằng dữ liệu mới, không dùng lại bản preview cũ.

Đưa càng vào, dừng rồi nâng:

```bash
bash scripts/open_pallet_pickup.sh insert-and-lift
```

Lệnh đưa càng chờ trạng thái mới xác nhận `ALIGNED`, pallet đã khóa, YOLO tắt và cửa sổ RGB-D 5 giây đạt trước khi gửi yêu cầu chạy **một lần**. Sau khi các trạng thái đạt, chờ thêm 0,4 giây để tránh ngắt do khởi động/discovery của terminal. Mặc định chờ tối đa 15 giây; nếu dữ liệu chưa đạt thì thoát, chưa gửi lệnh xe/càng. Ctrl+C trong lúc chờ hủy yêu cầu; dữ liệu phục hồi sau đó không tự chạy. Có thể đặt `--ready-timeout 20` (tối đa 30 giây). Bộ điều khiển vẫn kiểm tra lại phép đo, càng, laser, toàn bộ quỹ đạo và các giới hạn tại thời điểm nhận lệnh; CLI không dùng lại kết quả preview hay nới điều kiện chạy. Bản sửa CLI có hiệu lực ở lần gọi tiếp theo, không cần khởi động lại stack.

Tối ưu CPU trong `pallet_pickup_safety.py`: khi không có điểm laser trong hộp bao footprint đã đệm, bỏ các phép chiếu lên sáu cạnh rỗng; trường hợp có điểm trong hộp vẫn kiểm tra đủ đa giác. Không đổi footprint, đệm 3 cm hay thời hạn dữ liệu. Phần tối ưu trong bộ điều khiển được nạp ở lần mở stack tiếp theo.

Hoặc tách hai bước để quan sát:

```bash
bash scripts/open_pallet_pickup.sh insert
# Sau khi báo INSERTED, xe đứng yên và bạn kiểm tra càng đã vào:
bash scripts/open_pallet_pickup.sh lift
```

Giữ terminal lệnh đang chạy mở. Ctrl+C gửi dừng cả xe và càng. Mất/hard-kill terminal làm mất heartbeat và bộ điều khiển dừng, không tự tiếp tục khi terminal mở lại. Có thể dừng từ terminal khác:

```bash
bash scripts/open_pallet_pickup.sh stop
```

Nếu một lượt đưa càng bị chặn trước khi càng vào pallet, sau khi sửa nguyên nhân và giữ nguyên vị trí pallet, có thể kiểm tra/tiếp tục bằng:

```bash
bash scripts/open_pallet_pickup.sh resume-preview
bash scripts/open_pallet_pickup.sh resume-insert-and-lift
```

Phục hồi cần quỹ đạo đã chạy/dừng còn trong bộ điều khiển, hoặc checkpoint được nạp rõ ràng qua `PALLET_RECOVERY_CHECKPOINT` khi mở stack. Không tự phục hồi khi laser có lại. Khi quỹ đạo đã chốt còn được giữ và odometry liên tục, lệnh tiếp tục dùng chính pallet đã chốt, cho phép camera mất dấu vì pallet khuất ảnh; YOLO vẫn phải tắt và heartbeat nhận diện còn mới. Xe đứng yên, càng gần HOME và vẫn đạt ngang <2 cm/góc <2°. Chỉ hỗ trợ phục hồi khi gốc càng còn cách mặt pallet >=0,60 m. Điều kiện khởi đầu một lượt mới >=1,20 m không đổi. Đường còn lại vẫn kiểm tra đủ footprint/laser, phanh và hành trình. CLI yêu cầu odometry/laser mới liên tục 5 giây trước khi gửi yêu cầu chạy.

Checkpoint thường qua khởi động lại hợp lệ trong cùng lần bật Pi, tối đa 30 phút, và cần RGB-D hiện tại khớp pallet đã chốt trong 2 cm/2°, chiều rộng trong 2 cm, cao độ càng trong 3 mm. Checkpoint có `frozen_trajectory: true` chỉ dành cho khởi động lại có kiểm soát khi xe và pallet giữ nguyên: thời hạn 10 phút, cùng lần bật Pi, neo quỹ đạo theo odometry đứng yên đầu tiên và từ chối nếu odometry đổi gốc sau đó. Chế độ này không cần phép đo camera mới khi pallet đã khuất ảnh. Nhận diện/khởi động/nạp checkpoint không tự cho xe chạy.

Bản sửa laser giữ hàng đợi một mẫu ở đầu vào bộ lọc và bộ căn; bỏ mẫu cũ/quá hạn, giữ nguyên thời điểm thu thực. Không tăng thời hạn laser 300 ms hoặc gán lại stamp mới cho dữ liệu cũ.

Nút **Dừng xe và càng** trên web cũng gọi cùng chủ điều khiển. Nếu chỉ cần dừng HOME/động cơ nâng trực tiếp:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /lift/stop std_msgs/msg/Empty '{}'
```

Sau khi đã đưa càng/nâng, bộ điều khiển giữ vận tốc xe bằng 0 và không cho nút căn bắt đầu lại trong pallet. Lượt chạy tiếp cần đưa xe/càng ra ngoài theo thao tác được kiểm chứng; `reset` chỉ chấp nhận khi đo mới thấy gốc càng cách pallet ≥1,20 m, xe đứng yên và càng gần HOME. Chưa bổ sung tự lùi rút càng hoặc vận chuyển hàng trong thay đổi này.

## Đường đi và cao độ

- Bắt đầu yêu cầu `ALIGNED`, các phép đo mới đã bù RGB-D, YOLO tắt/xử lý xong, cửa sổ RGB-D 5 giây đạt, odometry/laser mới và càng đã HOME, đứng yên ở hành trình 0–1 cm.
- Chốt tâm lỗ và pháp tuyến trong `odom`. Với cấu hình chạy thật hiện tại, goal đưa càng là tâm xe ở trước mặt pallet một khoảng `fork_root_m + root_stop_m = 0,3655 + 0,020 m`. Đường thẳng từ vị trí xe tới goal sửa phần lệch ngang nhỏ còn lại; không quay tại chỗ trong pallet, không lùi.
- Chỉ nhận đường lọt khe theo **toàn bộ hình chiếu càng**, với biên bên 1 cm. Giới hạn đo/odom là ngang 2 cm và góc 2°, nhưng khe hẹp có thể đòi căn tốt hơn để mũi càng lọt. Đường, góc và khoảng cách phanh đều được kiểm tra trước/trong chuyển động.
- Khi còn ở xa và càng chưa qua mặt pallet, đường nông có thể sửa phần sai số còn lại với pháp tuyến. Không bắt tâm xe phải ở chính xác trên pháp tuyến ngay điểm đầu nếu Y/góc đã đạt. Ngay khi mũi càng qua mặt pallet, vẫn kiểm tra ngang với trục lỗ ≤2 cm và toàn bộ chiều rộng càng trong khe. Độ lệch khỏi đường đã chốt và cả dự báo phanh bị giới hạn 1,5 cm.
- Tốc độ lệnh tối đa 0,10 m/s, giảm dần gần goal theo khoảng cách phản ứng/phanh với dự trữ tốc độ 40%; kiểm tra cả quỹ đạo phanh theo lệnh và vận tốc đo vẫn giữ nguyên. Goal cấu hình chạy thật giữ gốc càng trước mặt pallet **2 cm**, chỉ chấp nhận vùng **1,5–2,5 cm**, giới hạn cứng gốc càng **1 cm** theo yêu cầu người vận hành khi đã đứng yên rồi mới nâng. Với càng dài 39 cm, chiều sâu đưa vào theo odometry khoảng **36,5–37,5 cm**. Đây là ước lượng từ mặt pallet đã chốt, không phải cảm biến tiếp xúc. Cấu hình này thay cho lượt đã dừng ở khoảng 5,1 cm: vào sâu thêm khoảng 3 cm nhờ giảm tốc và đổi goal, không cộng một lệnh tiến mù sau khi đã nâng. Có hiệu lực ở lần mở bộ điều khiển tiếp theo; không khởi động lại khi đang mang pallet chỉ để áp dụng thay đổi này.
- Goal nhận diện/pickup/dropoff đã lưu **không bị sửa**. Với `goal_policy: operator_taught_station_only`, cần TF `map → odom` mới; điểm tính từ RGB-D phải khớp pickup đã dạy trong 5 mm/1°, và trước nâng pose thực tế phải trong 1 cm/1° cùng các kiểm tra hình học/phanh. Không khớp thì dừng, không dùng điểm tương đối thay thế hoặc tiến thêm để đổi độ sâu. Chế độ cũ không có chính sách này vẫn dùng pickup làm tham chiếu với giới hạn bù RGB-D 15 cm/5°; stack độc lập không có AMCL/map báo rõ `pallet_relative_odom`.
- Số đo người vận hành: mặt dưới thanh ngang trên lỗ cao 10 cm; mặt trên càng HOME khoảng 7,3 cm. Dùng cao độ càng hiện tại lúc chốt, giữ ít nhất 8 mm ở phía trên theo mô hình. Mốc đáy lỗ ở sàn là giả định; không coi chiều cao tổng pallet là phép đo chiều sâu lỗ.
- Khi dừng ổn định ít nhất 0,8 giây và còn trong vùng kết thúc mới nâng. Mặc định **tăng hành trình càng 8 cm**, khoảng 8 giây ở giới hạn firmware 1 cm/s; cấu hình tại `config/pallet_pickup.yaml`. Đây là hành trình càng, không phải chiều cao pallet nhấc khỏi sàn. Node chỉ báo COMPLETE sau phản hồi MCU đạt cao độ và IDLE; không có cảm biến tải để xác nhận pallet thực sự được nâng.

## Dữ liệu trong lúc pallet khuất camera

Sau khi chốt, đoạn cuối dùng pallet cố định + odometry; không yêu cầu nhìn thấy lại chân pallet dưới xe và không chạy lại YOLO. Trạng thái LOST của phép đo khi pallet ra khỏi ảnh không được giả làm phép đo RGB-D mới. Người vận hành xác nhận vùng dưới/càng và quan sát lượt chạy như đã yêu cầu; bộ điều khiển không phát hiện được vật cản dưới mặt phẳng quét LiDAR hoặc pallet bị người khác dịch trong đoạn khuất ảnh.

Chuỗi lệnh duy nhất giữ nguyên:

```text
handle_pallet.py → /pallet/alignment/cmd_vel
  → Collision Monitor (laser, vùng dừng hiện có)
  → /cmd_vel_safe → cmd_vel_stamper → base_controller
```

Không giảm footprint hay vùng dừng laser. Ngoài Collision Monitor, kiểm tra footprint đầy đủ đệm 3 cm trên toàn bộ đường và quỹ đạo phản ứng/phanh. Chặn mất odometry 200 ms, laser 300 ms, phản hồi càng 350 ms, heartbeat detector 1 giây, heartbeat terminal 600 ms, odometry reset, sai lệch lớn hoặc có nguồn lệnh khác. SET càng cần phát liên tục; MCU tự dừng sau 350 ms mất lệnh. `/lift/stop` gửi STOP thật, ngắt heartbeat HOME và yêu cầu publisher SET cũ im ít nhất 400 ms trước khi nhận mục tiêu mới.

Quan sát `/pallet/handling/status`, Path `/pallet/handling/path` trong RViz frame `odom`. Trạng thái INSERTED/COMPLETE xác nhận kết thúc điều khiển theo odometry/phản hồi bước nâng, chưa phải xác nhận cơ khí bằng cảm biến dưới càng.

## Kiểm chứng

Kiểm thử hình học và trạng thái dùng nhiều mức phanh; kiểm thử ROS dùng Collision Monitor thật, base/càng giả trong **domain 219**, không khởi động driver hay gửi lệnh tới xe thật. Có luồng terminal `insert-and-lift`, mất camera gần pallet, mất laser/heartbeat, vật cản mới, insert riêng rồi lift riêng, dừng lúc nâng và Ctrl+C terminal. Trên xe thật ngày 2026-11-02, người vận hành đã tiếp tục thành công sau hai lần dừng bảo vệ: phản hồi COMPLETE, hành trình nâng 8 cm, gốc càng cách mặt pallet theo odometry khoảng 5,1 cm; người vận hành xác nhận xe và càng nâng đúng. Sau đó đã hạ về hành trình 0 cm, tiến thêm 2,96 cm và nâng lại 8 cm theo yêu cầu người vận hành. Trạng thái cuối COMPLETE, MCU IDLE/fault NONE, gốc càng còn cách mặt pallet theo odometry 2,11 cm, lệch ngang 4,8 mm và lệch góc 0,265°. Trong lượt tiến ngắn, watchdog dừng ở cách goal 1,14 mm trước khi nâng; đã giữ xe đứng yên và kiểm tra lại vùng kết thúc trước lệnh nâng riêng. Ngưỡng chuyển sang phanh hoàn tất nay là trong 2 mm (vẫn kiểm tra vùng kết thúc và đứng yên) để tránh kẹt ở tốc độ thấp gần goal. Kiểm thử phục hồi bổ sung xác nhận giữ quỹ đạo khi camera mất dấu và từ chối checkpoint hết hạn hoặc odometry đổi gốc.

Kết quả kiểm tra 2026-11-02: 24 kiểm thử hình học/trạng thái, 8 kiểm thử ROS đưa càng, 8 kiểm thử web và 10 kiểm thử bridge đạt. Một kiểm thử hồi quy với Nav2 thật/base giả cũng đạt toàn bộ các bước PLANNING → NAVIGATING → CHECKING → TURNING → ALIGNED khi dùng chủ điều khiển mới. Build hai package `tai_robot_one`/`sllidar_ros2`, `launch --show-args` và lệnh trợ giúp CLI đều đạt. Goal YAML giữ SHA-256 `dc339f0a18e175ae4ea4bbf31866d9b8a9677d45adb212916b84d38fe70d80f8`.

## Hạ rồi tiến lại sau lượt nâng đã hoàn tất

Chỉ khi xe và pallet giữ nguyên, còn quỹ đạo và odometry liên tục của lượt COMPLETE (hoặc checkpoint COMPLETE cùng phiên, tối đa 10 phút), có thể dùng:

```bash
bash scripts/open_pallet_pickup.sh lower-preview
bash scripts/open_pallet_pickup.sh lower
bash scripts/open_pallet_pickup.sh resume-preview
bash scripts/open_pallet_pickup.sh resume-insert-and-lift
```

`lower` giữ xe đứng yên, hạ về cao độ đưa càng đã chốt bằng phản hồi MCU; không HOME và không tự tiến. Chỉ sau trạng thái LOWERED mới cho phép lệnh tiếp tục riêng trong pallet. Đoạn tiến lại tối đa 3,5 cm, giữ hướng hiện tại để tránh xoay trong lỗ, kiểm tra toàn bộ thân/càng và phanh. Nếu hạ bị lỗi hoặc xe/càng dịch ngoài giới hạn thì dừng, không tự chuyển bước. Nhận diện RGB-D mới không bắt buộc cho đoạn này vì quỹ đạo pallet đã được giữ; laser, odometry, phản hồi càng, nguồn lệnh và heartbeat vẫn phải hợp lệ. Ctrl+C dừng cả xe và càng.

Checkpoint `lift_only` chỉ được dùng cho khởi động lại có kiểm soát tại vùng kết thúc sau dừng không tiến: cùng lần bật Pi, tối đa 10 phút, xe/càng giữ nguyên và odometry đứng yên liên tục ít nhất 0,8 giây. Chỉ lệnh `lift` riêng mới kiểm tra lại hình học, cao độ, laser và nguồn lệnh để nâng; nạp checkpoint không tạo chuyển động tiến hay nâng tự động.
