# Căn xe: tiến chéo tới pháp tuyến pallet, rồi sửa góc bằng RGB-D

Bộ chạy thật dùng `handle_pallet.py`, kế thừa bộ căn `stage_pallet.py`. Người vận hành mở stack rồi bấm **Bắt đầu căn xe** trên web; bước căn dừng ở ALIGNED và khoảng cách ≥1,20 m. Sau đó người vận hành có thể tự gọi lệnh đưa càng/nâng riêng theo [PICKUP.md](PICKUP.md). Khởi động stack và nhận diện không tự chạy bước đưa càng.

## Các goal nhiệm vụ cố định

Goal tới chỗ nhận diện (dự kiến khoảng 1,5–1,8 m), goal đưa càng vào nâng và goal hạ hàng là các goal nhiệm vụ cố định. Ba goal pallet 1 đã lưu trong `ros2_ws/config/pallet_goals.yaml`; pallet 2 sẽ cung cấp sau. Bộ căn chưa chọn trạm tự động và không ghi đè các goal đó. Goal căn chỉnh là goal trung gian tính từ pallet được nhận diện tại trạm đang dùng; không lấy tọa độ pallet cũ ở trạm khác.

Khoảng 1,5–1,8 m phải xác định mốc đo: camera, tâm xe và gốc càng có vị trí khác nhau. Ràng buộc trong bộ căn luôn dùng khoảng cách vuông góc **mặt pallet → gốc càng**, không suy ra an toàn từ con số khoảng cách camera.

### Goal đưa càng pallet 1 đã được kiểm chứng cơ khí

Ngày 2026-11-01, người vận hành xác nhận goal `pallet_1.pickup` trong `ros2_ws/config/pallet_goals.yaml` được ghi khi **càng đã nằm trong pallet xanh, chỉ còn bước nâng**. Đây là điểm dừng sau đưa càng, không phải điểm chờ trước pallet. Giữ tọa độ `(2.976802885580789, -0.19648969320318954)` và yaw `-90°` đã yêu cầu. Ban đầu người vận hành chỉ yêu cầu đưa càng; yêu cầu mới nhất đã cho phép **tới vị trí đưa càng, dừng rồi nâng pallet**. Không còn áp dụng lệnh cấm nâng cũ cho nhiệm vụ này; vẫn không nâng trước khi đưa càng thành công.

Bộ căn `stage_pallet.py` vẫn chỉ chạy tới khoảng cách chờ ≥1,20 m; nó chưa thực hiện đoạn đưa càng. Nav2 thường kiểm tra footprint 2D gồm thân và càng, nên có thể chặn đoạn càng đi vào pallet. Không tắt tránh vật cản hoặc thu nhỏ footprint chỉ để ép goal được chấp nhận.

Kiểm tra sau khởi động lại lúc khoảng 18:00: xe đứng yên, RGB-D báo mặt pallet → gốc càng khoảng 1,289 m; goal quy đổi qua định vị hiện tại cách xe khoảng 1,32 m. Sai khác suy ra khoảng 2,7 cm **không chứng minh goal đã lưu sai**. Độ lệch chuẩn AMCL báo sau 5 lần cập nhật đứng yên còn khoảng 8,5 cm theo X/Y và 3,7° theo yaw; lúc ghi pickup là khoảng 5 cm và 2,34°. Không dùng việc TF đứng yên làm bằng chứng định vị chính xác tới cm.

Phép kiểm tra 3D đầu tiên dùng khối mast đặc và carriage bắt đầu quá xa về sau. Báo cáo `data/diagnostics/pickup_precise_shape_audit_20261101.json` đã tách hai thanh mast/các thanh ngang và sửa carriage về X `[0.3305, 0.3655]`, Z `[0.069, 0.129]`. Vẫn còn điểm cloud trong vùng mast/carriage tại goal giả định; kích thước khung nâng còn là ước lượng và không có phản hồi `lift_joint` trong `joint_states` của lượt này. Không kết luận va chạm cơ khí chỉ từ các khối này, cũng không kết luận thông thoáng từ việc không có điểm trong vùng càng.

Hai đầu ra đang có bước hiệu chỉnh khác nhau: `/pallet/target` cộng bù vị trí suy ra (forward `+0.0384298849 m`, lateral `+0.0205314774 m`) của hồ sơ thử nghiệm; `/camera/depth/points_nav_corrected` sửa độ nghiêng depth và độ cao sàn, giữ X/Y sau chiếu, không áp dụng hai bù vị trí pallet đó. Không dùng trực tiếp cloud này như thể đã có cùng độ chính xác vị trí tuyệt đối với target. Người vận hành đã đối chiếu và xác nhận khoảng cách thực tế mặt pallet → gốc càng **1,29 m**, khớp RGB-D tại vị trí trước lượt tiến bên dưới. Không sửa bias khoảng cách để ép khớp goal bản đồ, không yêu cầu đo lại cùng khoảng cách này. Xác nhận một vị trí không xác nhận độ chính xác toàn miền hoặc pose AMCL.

Lượt tiếp cận giới hạn sau xác nhận: nhận diện lại khi xe đứng yên, YOLO tự tắt; cửa sổ 5 giây có 74/74 mẫu hợp lệ, khoảng 15,04 Hz. Chỉ gửi `FollowPath` trên đường thẳng 45 cm trong `odom` qua Nav2 thường, giới hạn lệnh 0,10 m/s. Trước chạy kiểm tra footprint đầy đủ tới 55 cm (gồm 10 cm dự phòng) và hướng ±6°. Giữ nguyên các costmap, footprint và Collision Monitor; không chạy BT tìm đường/vòng quanh goal. Nav2 trả `SUCCEEDED`/error 0 khi odometry khoảng 40 cm, xe dừng sau phanh ở khoảng **43,25 cm**; độ trôi ngang cuối khoảng 1 mm. Tốc độ odometry có lúc khoảng 0,12 m/s dù lệnh giới hạn 0,10 m/s; không coi giới hạn lệnh là tốc độ thực đo. Báo cáo `data/diagnostics/pickup_bounded_approach_20261101.json`.

Sau lượt này camera không còn thấy mặt trước/hai chân pallet: ảnh `data/diagnostics/pickup_after_approach_rgb_20261101.png` chỉ còn mặt trên pallet ở mép dưới. Nhận diện lại trả `REFINING: pallet clipped by recognition view`. Khoảng cách gốc càng còn khoảng 0,86 m **chỉ là dự báo từ snapshot trước chạy trừ odometry**, chưa có phép đo RGB-D mới hợp lệ. Xe đã dừng ngoài pallet; không được dùng thông báo thành công đoạn 45 cm làm kết quả đã đưa càng. Đoạn vào càng cần bàn giao snapshot pallet đã xác nhận sang bám quỹ đạo trong odom và kiểm tra riêng vùng đi qua lỗ; không chờ RGB-D nhìn đủ chân pallet ở cuối hoặc bỏ tránh vật cản để ép Nav2 thường đi vào. Chưa triển khai/kiểm chứng đoạn vào càng này.

Chưa hoàn tất và chưa chạy đoạn đưa càng. Báo cáo chỉ đọc mới: `pickup_live_geometry_20261101.json`, `pickup_precise_shape_audit_20261101.json`, `pickup_stationary_localization_20261101.json` trong `data/diagnostics/`. Không có lệnh nâng/hạ được phát trong các kiểm tra này.

Chuẩn bị cho yêu cầu nâng mới: `/lift/status` thực sự có phản hồi 10 Hz từ ESP32 chung IMU/càng; `use_lift=false` của ros2_control **không có nghĩa robot không có điều khiển nâng**. Đường nâng thật là `/lift_controller/commands` → `esp32_imu_bridge` → Serial `LIFT,SET,...`; `/lift/home` yêu cầu tìm công tắc đáy. `/joint_states` có nhiều publisher, nên một bản tin chỉ gồm bốn bánh không chứng minh thiếu `lift_joint`. Không mở thêm chủ Serial hoặc bật `use_lift=true` để tranh cổng ESP32 này.

ESP32 ban đầu báo `UNHOMED`, không có lỗi, cả hai công tắc chưa kích hoạt. Khi xe đứng ngoài pallet, đã gửi HOME qua bridge; lượt đầu không xác nhận được trong 1 giây nên đã cắt heartbeat bằng cách tạm dừng bridge >350 ms (watchdog firmware) rồi khôi phục. Lượt kế tiếp theo dõi 3 giây chờ bắt đầu đã hoàn tất về công tắc đáy: `IDLE`, `homed=true`, `lower_active=true`, `upper_active=false`, `position=0`, `fault=NONE`. Đây là **hạ càng tìm mốc**, chưa phải nâng pallet. Càng hiện ở mốc thấp nhất, khác cao độ trước HOME; cần kiểm tra cao độ phù hợp lỗ trước đưa vào. Không có lệnh `SET` nâng nào được phát. Báo cáo `data/diagnostics/lift_home_pre_pickup_20261101.json`.

Kiểm tra tiếp cận sau lượt 43 cm: `GridBased` trả đường 19 pose, nhưng `SE2Fallback` trả error `207`, không có đường footprint hợp lệ. Không gửi FollowPath đi vào bằng đường GridBased chưa được kiểm tra footprint. Theo TF hiện tại và snapshot RGB-D cũ trong odom (giả thiết pallet đứng yên), goal pickup quy đổi lệch trục lỗ khoảng **5,27 cm**, gốc càng dự kiến vượt mặt trước khoảng **2,44 cm**. Đây là bất đồng giữa goal bản đồ/định vị hiện tại và snapshot, không phải bằng chứng goal đã lưu cơ khí sai. Không sửa tọa độ goal hoặc ép chạy qua điều kiện này. `data/diagnostics/pickup_after_home_goal_check_20261101.json` xác nhận không có goal di chuyển đang chạy, bánh đứng yên và càng đã HOME, chưa tới pickup/chưa nâng pallet.

## Cách chọn goal

Tâm lỗ `C`, hướng vào pallet `n = (cos θ, sin θ)` và vị trí tâm xe `B` đều trong `odom`:

```
d_nearest = dot(C - B, n)
d_goal = max(d_nearest, fork_root_m + 1.20 + 0.10)
B_goal = C - d_goal * n
heading_goal = θ
```

Hình chiếu thuần ngang ở trên chỉ là mốc hình học: xe vi sai phải quay gần 90° để tới đó. Sau lượt thử thật, bộ chọn goal đã đổi sang dùng phần dư khoảng cách để **tiến chéo với góc nhỏ**:

```
d_min = fork_root_m + planning_clearance_m(config)
a_max = min(d_nearest - d_min, 0.80)
B_goal(a) = C - (d_nearest - a) * n,  0 <= a <= a_max
```

Lấy các điểm cách nhau 10 cm, gồm cả biên `a_max`; xếp theo tổng góc quay đầu/cuối nhỏ nhất rồi quãng đường. Kiểm tra từng điểm/đường với costmap mới và footprint đầy đủ; góc đi chéo tính từ hình học của từng goal, **không giới hạn cố định 15°**. Ví dụ lệch ngang thân xe khoảng 11 cm, góc mặt pallet 1,2°, gốc càng cách khoảng 1,63 m: có thể chọn goal tiến khoảng 33 cm theo pháp tuyến, đi chéo khoảng 36 cm với góc khoảng 23–25° và còn cách 1,30 m. Khi đã ở trên trục, không ép xe tiến thêm. Không có goal/đường hợp lệ thì dừng; không đi lùi, quay ngang 90° hoặc quay vòng để bù ngang.

Gốc càng cách `base_link` **0,3655 m**, mặt trước thân cách tâm xe **0,3005 m**, XY của `base_link` và `base_footprint` trùng nhau. Goal chừa ít nhất **10 cm** và tự cộng đủ dự phòng phanh/độ trễ: `max(clearance_margin_m, 0.02 + 0.2*v_max + v_max²/(2*a_brake) + 0.02)`. Với cấu hình hiện tại, khoảng cách goal tối thiểu là **1,3225 m**. Khoảng cách đo mới và toàn bộ quỹ đạo phanh vẫn phải đạt giới hạn 1,20 m.

Nếu đường/footprint tới điểm đầu không hợp lệ, thử điểm tiến chéo tiếp theo trong phần dư khoảng cách. Giới hạn goal cách xe 2 m, gốc càng cách pallet không quá 3 m, đường không dài quá 3 m. Gốc càng đang dưới 1,20 m: không bắt đầu, yêu cầu đặt lại xe. Điểm và đường phải nằm trong vùng đã quan sát của costmap; không đi qua ô chưa biết.

Nav2 dùng planner SE2 lattice có footprint đầy đủ, không cho lùi. Khi toàn bộ đường quay–đi chéo thẳng–quay thông thoáng, rút gọn đường lattice thành đường này và kiểm tra lại đủ các bước quay/tiến. Khi chạy, hướng xe phải nằm trong biên hướng của **chính đường đã kiểm tra**, cộng dung sai bám 0,04 rad; không lấy giới hạn đo góc mặt pallet 18° làm giới hạn quay của Nav2. Đường dự phòng cũng phải đi về phía trước mặt pallet, không quay ngang/ra sau. Giới hạn đo RGB-D 18° vẫn áp dụng khi lấy phép đo trước/sau di chuyển và sửa góc cuối. Trong đoạn Nav2, dùng snapshot pallet + odometry + laser; nếu camera mất dấu thì dừng ở goal và nhận diện lại khi xe đứng yên, không dùng phép đo ảnh cũ làm phép đo mới. Bộ căn kiểm tra toàn bộ phần bên trong footprint, các đoạn nối và quỹ đạo quay, cách tối đa 2 cm/2° giữa các mẫu. Điểm cuối lưới 5 cm được nối tới goal chính xác và kiểm tra lại. Controller riêng dùng ngưỡng xoay trước khi tiến 0,04 rad để bám đường chéo bằng đoạn thẳng. Không có đường hợp lệ thì dừng, không giảm footprint để tìm đường.

## Luồng thực hiện

1. Người vận hành đưa xe tới chỗ nhận diện và nhận diện đúng pallet. YOLO tự tắt; yêu cầu cửa sổ RGB-D ổn định 5 giây trước khi bắt đầu.
2. Bấm bắt đầu: xác nhận YOLO tắt, bỏ phép đo cũ, chờ ít nhất 3 phép đo mới và odometry/lidar/costmap hợp lệ.
3. Chốt tâm/hướng pallet trong `odom`, chọn goal và gọi Nav2 `ComputePathToPose` → `FollowPath`. Nav2 này chạy cục bộ trong `odom`, không cần AMCL/map hoặc biết goal nhiệm vụ cố định.
4. Khi Nav2 quay/di chuyển, pallet có thể ra khỏi camera. Đoạn này dùng **snapshot pallet đã chốt + odometry + laser/costmap**, không dùng số đo ảnh cũ như ảnh mới. Giới hạn 60 giây; mất odometry, laser, heartbeat YOLO tắt hoặc lớp bảo vệ thì hủy và dừng. Pallet phải đứng yên trong đoạn này.
5. Tới goal: đưa vận tốc về 0, chờ xe đứng yên, bỏ phép đo trước đó và chờ RGB-D mới ổn định 5 giây. Nếu mất dấu, dừng với `REACQUIRE_REQUIRED`; bấm **Nhận diện lại**, đợi ổn định rồi **Bắt đầu căn xe** tiếp. Không tự bật YOLO trong khi di chuyển.
6. Nếu đã đạt đồng thời ngang Y trong hệ xe <2 cm và góc <2°, giữ xe đứng yên để xác nhận. Nếu chưa đạt, dùng sai số ngang với pháp tuyến để tính lại goal khi ≥1,5 cm, tối đa 2 lượt sửa bổ sung. Khi ngang đủ nhỏ, chỉ quay tại chỗ theo góc mặt pallet; vận tốc tiến bằng 0.
7. Hoàn tất khi **|Y| < 2 cm**, **|góc| < 2°**, khoảng cách vuông góc **≥1,20 m**, xe đứng yên và điều kiện giữ liên tục 0,8 giây. Dừng ở đây.

Nếu bắt đầu khi xe đã ở trên trục pháp tuyến (sai số pháp tuyến ngang <1,5 cm), hoặc đã đạt cả hai ngưỡng Y/góc đang hiển thị, và đủ 1,20 m, bỏ bước Nav2: giữ nguyên vị trí, chờ số đo mới ổn định rồi sửa góc nếu cần. Không ép tạo goal tiến với đệm 1,3225 m khi xe đã đạt ngưỡng và không cần tiến. Ví dụ ngày 2026-11-02: gốc càng cách 1,277 m, Y khoảng -0,6 cm, góc khoảng -1°; sai số pháp tuyến của tâm xe có thể trên 2 cm do góc, nhưng hai tiêu chí căn đã đạt. Bước đưa càng vẫn lập/kiểm tra đường nông tới trục và kiểm tra toàn bộ càng lọt khe, không ép ALIGNED bằng lệnh tay. Không bắt xe trở lại X của goal, vì khoảng cách dọc là mức tối thiểu. Nếu Nav2 yêu cầu lùi sau lượt xoay cuối, bộ căn không phát vận tốc âm: khi đã đạt trục và còn ≥1,22 m thì dừng ở `REACQUIRE_REQUIRED` để nhận diện lại và chỉ sửa góc; trường hợp còn lệch trục hoặc quá gần thì vẫn chặn chạy.

Góc dùng `entry_heading_rad = -face_yaw_rad`, lấy từ mặt trước pallet, không dùng góc hướng tới điểm tâm. Dùng odometry nội suy tại thời điểm chụp để đưa phép đo vào `odom`. Các scalar vị trí đã tính theo thân xe; không biến đổi như điểm XYZ optical dù frame nguồn ảnh là `camera_color_optical_frame`. Chạy thật yêu cầu dấu `target_bias_applied`.

## Mở trên các terminal

Nếu camera chưa mở, trong terminal đầu tiên:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
ASTRA_POINT_CLOUD=false bash scripts/open_registered_astra.sh
```

Giữ camera đang chạy. Terminal thứ hai mở nhận diện/RGB-D, với hồ sơ bù vị trí và phương pháp góc đã kiểm chứng:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 \
  --target-color green \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml \
  --heading-floor-reference --start-paused
```

Terminal web:

```bash
bash /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra/scripts/open_pallet_web.sh
```

Terminal chạy thật (không mở trùng Nav2/hardware/teleop cũ):

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_alignment_real.sh
```

Script build bản robot/lidar trong `data/alignment_robot_ws`, mở bánh xe, IMU/EKF, lidar, bộ tránh vật cản, Nav2 riêng và bộ căn. Không dùng càng nâng hoặc CM029. Trên web `http://localhost:8765`, F5 để cập nhật giao diện, bấm **Nhận diện lại**, chờ RGB-D ổn định, rồi **Bắt đầu căn xe**. Tốc độ giới hạn **0,10 m/s**, quay **0,24 rad/s** (bộ điều khiển quay tải hiện có dùng mức hoạt động tối thiểu 0,22 rad/s). Nếu thiếu dữ liệu/đường hợp lệ, nút chưa cho bắt đầu hoặc bộ căn báo lý do.

**Dừng căn xe** hủy goal Nav2 và đưa lệnh về 0. Ctrl+C terminal chạy thật tắt stack. Không chạy lệnh căn `align_pallet.py --execute` cũ đồng thời với bộ mới.

## Bảo vệ và quan sát

Đường lệnh duy nhất:

```
/pallet/nav_cmd_vel (Nav2) hoặc quay RGB-D
  → stage_pallet.py kiểm tra/trộn theo trạng thái
  → /pallet/alignment/cmd_vel
  → pallet_alignment_collision_monitor
  → /cmd_vel_safe → cmd_vel_stamper → /base_controller/cmd_vel
```

Monitor kiểm tra đúng lifecycle ACTIVE, tham số vùng dừng và nguồn lệnh độc quyền. Mỗi lần trước khi phát lệnh, bộ căn kiểm tra footprint và khoảng cách trên toàn bộ quỹ đạo phản ứng/phanh. Laser quá 300 ms, odometry quá 200 ms, costmap quá 600 ms, mất heartbeat YOLO tắt hoặc có nguồn lệnh cạnh tranh đều chặn chạy. Khi căn góc, phép đo RGB-D quá 250 ms đưa lệnh về 0. Phản hồi nhận goal đến trễ sau nút dừng bị hủy, không được mở lại cổng vận tốc.

Quỹ đạo dự báo đã đổi từ ngoại suy vận tốc không đổi 1 giây sang **phản ứng 200 ms + toàn bộ quá trình phanh về 0**, kiểm tra cả vận tốc lệnh và vận tốc đo odometry. Dùng giảm tốc bảo thủ của bộ căn 0,08 m/s² và 0,40 rad/s², chậm hơn giới hạn controller thật 0,50 và 1,50. Ví dụ ở 0,10 m/s, quãng phản ứng+phanh là 8,25 cm. Kiểm tra vị trí gốc càng theo từng góc thực tế trong quỹ đạo và giữ thêm 2 cm trên giới hạn 1,20 m; vẫn kiểm tra đầy đủ thân/càng với ô vật cản và ô chưa biết. Ngoại suy vận tốc không đổi trước đây có thể vượt qua một goal đã an toàn dù controller sẽ dừng ở đó.

Kiểm tra toàn bộ phần trong footprint giữ nguyên thân xe/càng, đệm 3 cm và dự phòng ô lưới, chặn ô vật cản thật (OccupancyGrid 100) và ô chưa biết (-1). Giá trị 99 là lớp inflation đã cộng bán kính nội tiếp của xe; không dùng nó như vật cản thật cho mọi ô trong footprint vì sẽ cộng bán kính lần hai. Inflation vẫn được giữ trong planner/controller Nav2. Sửa này xử lý trường hợp Nav2 tìm được đường nhưng bộ kiểm tra phụ từ chối toàn bộ các lượt quay vì lớp đệm. Giao diện hiển thị kết quả căn hiện tại thay cho thông báo nhận yêu cầu bắt đầu đã cũ.

Đã kiểm tra sửa inflation ngày 2026-11-01: 14 kiểm thử hình học/trạng thái và 1 kiểm thử Nav2 xem trước trong domain riêng đều đạt. Với costmap thật đang chạy, chỉ gọi ComputePathToPose, không gọi FollowPath: Nav2 trả đường hợp lệ; kiểm tra cũ từ chối, kiểm tra sửa chấp nhận đường tới goal dịch ngang khoảng 5,8 cm. Khoảng cách mặt pallet → gốc càng thấp nhất trên đường kiểm tra là 1,5888 m; vận tốc quan sát bằng 0. Báo cáo lưu tại `data/diagnostics/staging_inflation_fix_20261101.json`. Đây là kiểm tra đường, chưa phải xác nhận kết quả căn khi xe chạy thật.

Để tránh ngưỡng truyền động bị ì đã ghi trong cấu hình robot, Nav2 dùng mức tiến hoạt động 0,10 m/s và quay hoạt động từ 0,22 rad/s, với cổng giới hạn gia tốc; vẫn phải đo sai số dừng ở lượt thử vật lý.

Laser 2D bảo vệ vật cản trong mặt phẳng quét; bản Nav2 cục bộ này chưa tích hợp vật cản thấp từ depth. Ngưỡng khoảng cách dựa trên ước lượng vị trí/hướng pallet và odometry, cần đối chiếu thước ở lượt thử vật lý. Camera đủ khoảng cách nhưng bị che hoặc sàn không phù hợp phương pháp góc thì không được chuyển sang quay chính xác.

RViz Fixed Frame `odom`, thêm Pose `/pallet/alignment/goal`, MarkerArray `/pallet/alignment/markers`, Path `/pallet_nav/plan`, Map `/pallet_nav/global_costmap/costmap`. Xem JSON ở `/pallet/alignment/status`; `fork_plane_distance_m` là khoảng cách mặt → gốc càng, `errors.distance_m` là phần dư so với mức tối thiểu 1,20 m, không phải yêu cầu tiến để đưa về 0.

Để chỉ kiểm tra đường mà không điều khiển xe, chạy Nav2 riêng và `open_pallet_staging.sh` (không `--execute`), với TF/odom/laser đang có. Bản xem trước chỉ gọi lập đường; không gửi `FollowPath` hoặc tạo publisher tới chuỗi động cơ. `open_pallet_alignment.sh` vẫn là bộ xem lệnh thuật toán cũ, không dùng để đánh giá goal mới.

## Kiểm thử

Lượt thử thật ngày 2026-11-01 đã đi chéo tới trục pallet, dừng yêu cầu lùi của Nav2, nhận diện lại khi đứng yên rồi chỉ xoay sửa góc. Sau khi dừng, `/pallet/alignment/status` báo `ALIGNED`, vận tốc bằng 0; RGB-D mới đo ngang khoảng 1,61 cm, góc đã lọc khoảng 0,41°, mặt pallet → gốc càng khoảng 1,289 m, cửa sổ 5 giây có 75/75 mẫu hợp lệ, khoảng 15 Hz. Báo cáo `data/diagnostics/pallet_aligned_stationary_20261101.json` và trace `pallet_adaptive_real_final_20261101.json`. Đây là một lượt thử tại vị trí hiện tại, chưa xác nhận tự động toàn bộ chuỗi cho mọi vị trí/pallet khác.

Sau khi đổi sang tiến chéo ngày 2026-11-01: 16 kiểm thử hình học/trạng thái và 6 kiểm thử ROS/Nav2 trong domain riêng đạt. Bao gồm căn tới đích bằng base mô phỏng, hủy lệnh, mất laser, ô chưa quan sát, chế độ xem trước và dừng khi hướng thực tế vượt biên 18°. Lượt mô phỏng đạt ngang khoảng 0,8 mm, góc 0,16°, quay nhiều nhất 14,43°, khoảng cách mặt pallet → gốc càng luôn ≥1,431 m. Replay costmap đã ghi trước cú quay thật cho đường chéo 29,45 cm, góc -11,16°, khoảng cách thấp nhất 1,30 m. Báo cáo `data/diagnostics/staging_diagonal_simulation_20261101.json` và `staging_diagonal_replay_20261101.json`; chưa chạy thử chiến lược tiến chéo trên xe thật.

Kiểm thử động học, footprint và tích hợp ROS thực hiện trong **các domain riêng 220–224 và 231**, với base giả. Không có driver động cơ trong kiểm thử. Kết quả này không thay thế lượt thử robot vật lý.

```bash
source /opt/ros/jazzy/setup.bash
source data/alignment_robot_ws/install/local_setup.bash
source ros2_ws/install/local_setup.bash
ROS_DOMAIN_ID=231 .venv/bin/python -m unittest discover -s tests -v
```

Kết quả kiểm tra ngày 2026-11-01: bộ tổng hợp 97 kiểm thử đạt; sau thay đổi xử lý dừng/phanh, chạy lại 17 kiểm thử căn mới và đều đạt. Có Nav2 planner/controller và Collision Monitor thật, base mô phỏng, dịch ngang 15 cm, pallet tạm ra khỏi ảnh, lấy phép đo mới rồi căn góc; xác nhận đạt ngưỡng, hủy FollowPath khi dừng/mất laser và từ chối ô chưa quan sát. Build `tai_robot_one`/`sllidar_ros2` và kiểm tra launch bằng `--show-args` đạt. Chưa chạy chuyển động robot vật lý.


## Paused by operator — 2026-11-01

Operator asked to stop work until tomorrow. No automatic continuation or motion.
Latest pause telemetry: `data/diagnostics/paused_insertion_20261101.json`.

- Opening ceiling (floor to underside of upper beam) confirmed by operator: 100 mm. Fork top at HOME is approximately 73 mm; upper clearance approximately 27 mm. Opening bottom at floor is an assumption, not an additional operator measurement.
- HOME completed earlier; forks remain at HOME. Fork insertion and pallet lifting have NOT completed.
- A checked normal-Nav2 reverse returned the robot approximately 407 mm to regain full pallet visibility. It is now around odom XY (0.0264, -0.0096), not at the last forward-approach pose. See `data/diagnostics/pickup_return_for_view_20261101.json`.
- Fresh green-pallet acquisition succeeded subsequently, with YOLO stopped and stable RGB-D around 15 Hz. Snapshot and replay: `data/diagnostics/insertion_preflight_20261101.{json,npz}`. These are historical snapshots; reacquire before any future motion.
- Saved pallet goals remain unchanged. Preflight proposed a local endpoint correction around 10.7 cm and entry lift height 5 mm; these are proposals ONLY, no SET or insertion command was sent.
- New insertion geometry / scene modules and stationary collector are under development. Sixteen unit tests pass. No integrated insertion controller, execution launch, or validated real insertion mode exists yet; do NOT advertise an execution command.
- Current blocker: replay initially intersects low floor samples at the wheel envelope. Wheels now use cylindrical geometry. A proposed ground classifier passes synthetic tests but rejects the real snapshot because fitted floor disagrees with its calibration limits. This rejection is unresolved; do NOT loosen limits or discard floor points simply to force a pass. Inspect floor fit / original calibration and validate on replay before further integration or physical insertion.
- `scipy>=1.12,<2` added to perception requirements and installed in its venv for spatial matching.
- Normal hardware/navigation/perception services have not been shut down merely by this pause; verify current processes and fresh telemetry on resumption. No motion goal should remain active.
