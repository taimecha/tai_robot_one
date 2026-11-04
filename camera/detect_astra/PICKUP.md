# Lấy pallet bằng phép đo mới và đường thẳng odometry

Cập nhật 03-11-2026. Người vận hành tự chạy lệnh. Chưa thử phương pháp mới trên xe thật.

## Kích thước khoảng trống đã xác nhận

Pallet có mặt trên và **bốn trụ tại bốn góc hình chữ nhật**. Trụ trước bên trái
thẳng hàng với trụ sau bên trái; bên phải tương tự. **Không có thanh chắn ngang
hay vách kín phía sau.** Khoang giữa hai hàng trụ rộng 25,5 cm, dài theo hướng
càng 35,5 cm và cao 10 cm. Khoảng trống từ mặt sau pallet tới vật cản ngoài là
30 cm (`rear_clearance_m: 0.300`). Đã lưu trong `config/pallet_pickup.yaml`.

Nhìn từ trên xuống (dấu ● là trụ, mũi tên là hướng hai càng):

```text
         Phía sau hở
     ●                 ●
           ↑     ↑
           ↑     ↑
     ●     ↑     ↑     ●
               Xe
```

Bộ kiểm tra giữ toàn bộ vùng quét hai càng trong hành lang giữa hai hàng trụ,
bao gồm khi đầu càng đi qua cặp trụ sau. Nó không dựng vách ở cuối chiều dài
35,5 cm. Vùng kiểm tra ngang liên tục là mô hình bao bảo thủ của hai hàng trụ,
không phải phép nhận diện riêng từng trụ sau bằng camera.

Càng dài 39 cm; gốc càng mục tiêu cách mặt trước **2 cm**. Khi thẳng hoàn toàn,
phần càng vào qua mặt trước khoảng **37 cm**, nhô qua mốc 35,5 cm khoảng **1,5 cm**.
Giới hạn thân xe, góc, lệch ngang và bao phanh vẫn áp dụng. Khoảng chừa 2 cm
`rear_margin_m` tính trước **vật cản ngoài phía sau**, không phải trước cặp trụ sau.
Nếu không khai báo khoảng trống phía sau, chương trình không tự cho phép dùng
vùng ngoài pallet.
Khoảng trống 30 cm là thông tin của vị trí đặt hiện tại, cần kiểm tra lại nếu
chuyển pallet tới nơi có vật cản khác phía sau.

Các số 33,5 cm / gốc càng 6,17 cm trước đó thuộc giả thiết khoang dừng tại mặt sau;
chúng đã được thay thế sau khi người vận hành xác nhận hình học bốn trụ hở sau.

Bề rộng 25,5 cm là giới hạn trên cho phép đo trực tiếp RGB-D: nếu camera đo khe
hẹp hơn, vẫn dùng giá trị nhỏ hơn. Không nới khe để cho quỹ đạo đi qua. Cần cập
nhật kích thước nếu đổi pallet hoặc hình học lỗ thay đổi.

## Kiểm tra quỹ đạo và odometry

Theo yêu cầu người vận hành: nếu lệch ngang **ban đầu** nhỏ hơn ngưỡng trong
`pallet_staging.yaml` (hiện 1,5 cm), nhưng góc còn ngoài ngưỡng và không quá
12°, bộ căn chốt chế độ **chỉ quay bù góc**. Lệnh tiến luôn bằng 0; không dùng
đường chéo, không tự chuyển sang sửa ngang khi camera đổi số ngang sau quay.
Đích quay là hướng pallet đã đo, không cộng thêm góc ngắm tâm. Sau khi góc
còn dưới 0,5° và xe dừng ổn định, đo lại ngang. Nếu cả ngang/góc đạt thì
`ALIGNED`; nếu ngang vượt ngưỡng thì `YAW_CORRECTED`, kết thúc lượt căn và
chu trình giữ dừng trước đưa càng. Mất dấu nhận diện lại vẫn giữ chế độ chỉ
quay trong lượt đó; không tự lập đường dịch ngang.

Đường căn ban đầu chỉ chọn góc quay danh định tối đa **12°** so với hướng xuất
phát và hướng pallet. Đường lattice đi vòng hoặc quay lớn cũng bị loại. Nếu
quãng tiến ngoài khoảng cách bảo vệ không đủ để dịch ngang bằng góc nhỏ,
chương trình dừng và báo cần vị trí nhận diện xa hơn; không tự lùi hoặc nới
kiểm tra vật cản. Sai số góc mặt pallet vài độ không đồng nghĩa phép chiếu
ngang ngắn cũng chỉ cần quay vài độ.

RViz kiểm tra vật cản đã bỏ các đường bao đỏ/vàng và nhãn quỹ đạo từ log cũ.
Giữ đường bao thân/càng xanh lá, LiDAR và vật cản RGB-D; thay đổi hiển thị
không thay đổi Collision Monitor hay kích thước vùng bảo vệ khi chạy thật.

Kiểm tra toàn bộ vùng quét quỹ đạo chạy trong luồng riêng trên bản chụp pose,
pallet và costmap; luồng ROS vẫn nhận odom/laser và giữ lệnh dừng trong lúc tính.
Trước khi gửi `FollowPath`, bộ căn kiểm tra lại dữ liệu mới, trạng thái bảo vệ,
xe chưa dịch khỏi pose bắt đầu và vùng vật cản của costmap còn phù hợp. Costmap
đổi thì kiểm tra lại; lệnh dừng/hủy loại kết quả cũ. Giữ ngưỡng odometry 200 ms.

Trong bước checkpoint/đưa càng, điểm laser được đổi sang odom bằng pose nội
suy từ `/odom` tại đúng thời điểm chụp và TF LiDAR → base tại thời điểm đó.
Không phụ thuộc bản TF odom động đến sau dữ liệu odom. Nội suy cần hai mẫu
bao quanh, cách nhau tối đa 150 ms; không ngoại suy và không lấy pose mới nhất
thay cho pose tại thời điểm chụp. Ưu tiên scan mới nhất có pose hợp lệ, vẫn giữ
giới hạn tuổi nguồn và thời gian nhận laser 400 ms.

Nếu pose/TF tại thời điểm chụp chưa tới, phát tốc độ 0 và tiếp tục nhận callback
tối đa 120 ms trong cùng lượt đang chạy. Có dữ liệu hợp lệ thì dùng lại quỹ đạo
đã chốt; quá hạn thì dừng lượt. Laser/odom quá hạn, hình học sai hoặc vật cản
vẫn chặn ngay; khoảng chờ TF không mở rộng các hạn dữ liệu này. Không tự tiếp
tục một lượt đã báo BLOCKED/STOPPED.

## Chuỗi mới

### Khi lệch ngang chưa đạt nhưng goal nhận diện đã ở gần pallet

Giữ đúng goal nhận diện đã lưu; không thêm backoff. Nếu phép chiếu staging
không có quãng tiến để sửa ngang bằng góc nhỏ trước mốc 1,20 m, handler có thể
kiểm tra **đường riêng tới checkpoint 1,10 m**. Trạng thái `CHECKPOINT_READY`
chỉ xác nhận đường ngoài pallet, không đồng nghĩa `ALIGNED` hay cho phép nâng.
Chỉ áp dụng khi staging không có candidate hình học; không chuyển một lỗi
va chạm/unknown, mất dữ liệu hay lượt chỉ quay thành đường mới.

Điều kiện: bắt đầu gốc càng ≥1,20 m, lệch trục ≤15 cm, góc ban đầu so với
pallet ≤12°; góc quay tới đường và hướng đường so với pallet ≤20°, đủ cao/khe.
Mốc 20° chỉ dùng cho đoạn ngoài pallet tới checkpoint 1,10 m, không áp dụng
cho đưa càng. Kiểm tra toàn bộ đường bao thân/càng
và khoảng chừa 21 cm trên các điểm laser; chạy tối đa 0,08 m/s, lệch đường
và quỹ đạo phanh tối đa 1,5 cm. Quay nhỏ theo đường, tiến tới checkpoint,
dừng rồi quay theo hướng pallet. Gốc càng luôn ≥1,075 m trong đoạn này,
đầu càng chưa đi qua mặt pallet. Giữ các kiểm tra sensor/CM/heartbeat đang có.

Sau đó bắt buộc đo lại RGB-D 5 giây, chốt đường thẳng và kiểm tra toàn bộ
vùng quét hai càng trước khi vào pallet. Ngưỡng đoạn cuối vẫn 5 mm/0,5°.
Không có phép đo mới đạt, lệch đường, có vật cản hoặc không có đường đưa
càng thẳng thì dừng ngoài pallet, không nâng. Sau `INSERTED`, người vận hành
vẫn kiểm tra và gõ `nang` trước khi nâng 8 cm.

Mỗi phiên nhận diện xong sẽ in ngay khoảng cách gốc càng tới mặt pallet (vuông
góc mặt và dọc xe), tọa độ tâm lỗ theo xe, lệch ngang trái/phải, góc cần bù và
bề rộng khe. Số đo dùng dữ liệu mới đã bù; lưu cùng log trong
`recognition_results.jsonl`. Việc in này cũng áp dụng cho phiên đo lại tại
checkpoint.

1. Nhận diện riêng pallet xanh, YOLO tắt; RGB-D, laser, odom và càng đạt điều kiện.
2. Bộ căn dùng Nav2 trong odom để đến vùng nhận diện/căn, gốc càng cách pallet ít
   nhất 1,20 m; ngưỡng ban đầu ngang 1,5 cm, góc 2°.
3. Tiến tới điểm đo lại: gốc càng cách mặt trước khoảng **1,10 m**. Đầu càng còn ngoài.
   Mốc 80 cm trước đây đã bị bỏ sau ảnh thật cho thấy hai chân ra khỏi mép dưới.
   Trong chu trình có heartbeat, bắt buộc mở một phiên nhận diện MỚI khi xe
   đứng yên tại checkpoint; không chờ bộ bám ảnh từ vị trí xa tự hồi phục.
   Khi đó chỉ cho phép YOLO trong trạng thái `CHECKPOINT_RECOGNITION`, lệnh
   tốc độ 0, pose giữ trong 5 mm/0,5°, cảm biến/càng/CM/heartbeat còn hợp lệ.
   Quyền nhận diện hết hạn nếu mất trạng thái giữ dừng 0,6 giây. Phiên mới
   phải tăng số session, thấy hai chân, YOLO tắt và RGB-D khỏe; vị trí pallet
   không được đổi quá 8 cm/5° so với trước tiếp cận. Không đạt trong 45 giây
   thì dừng lượt; không đưa càng theo phép đo cũ.
4. Dừng; chỉ dùng các ảnh chụp **sau khi dừng**. Cần cửa sổ mới liên tục ít nhất
   5 giây, ít nhất 10 mẫu/giây, không gián đoạn trên 250 ms, vị trí dao động không
   quá 2 cm và góc không quá 1°. Ảnh không đạt thì giữ dừng và hết hạn sẽ hủy.
5. Căn góc chậm khi đầu càng còn cách mặt pallet trên 10 cm. Sau mỗi lượt xoay,
   lấy lại đủ cửa sổ đo đứng yên trước khi chốt đường.
6. Tính một đường thẳng theo hướng thực tế của xe trong `odom`, kiểm tra toàn bộ
   hình chữ nhật hai càng trong lỗ và chiều sâu trước khi chạy. Không dùng goal
   pickup bản đồ, không sửa ngang bằng đường chéo trong đoạn cuối.
7. Bám đúng đường đã chốt bằng EKF odometry có IMU; tốc độ tối đa 0,10 m/s và giảm
   dần khi gần điểm dừng. Sai lệch khỏi đường 5 mm hoặc góc so với đường trên
   0,5° làm hủy; kiểm tra thêm hướng so với pallet, khe lỗ và bao phanh.
   Không xoay tại chỗ ở trong pallet. Mất ảnh khi đã chốt đường cuối không làm
   thay đổi goal; mất odom, laser, phản hồi càng hoặc heartbeat sẽ dừng.
8. Xe dừng ít nhất 0,8 giây và đạt hình học kết thúc, báo `INSERTED`.
   **Chưa nâng.** Kiểm tra thực tế hai càng không tì chân/thanh, vùng nâng trống,
   rồi gõ `nang` và Enter trên terminal chu trình. Lúc đó mới nâng hành trình 8 cm.

5 mm / 0,5° là ngưỡng giám sát theo cảm biến, không phải bảo đảm độ chính xác xe
thật hoặc xác nhận có tải. `COMPLETE` xác nhận hành trình càng theo MCU; vẫn cần
quan sát pallet. LiDAR ở cao không tự xác nhận vị trí chân trụ thấp hoặc vật cản thấp phía sau pallet.

## Khởi động sau khi tắt Pi

Chạy trên Pi qua SSH. Khi chưa có pallet chỉ mở phần cứng nếu cần kiểm tra;
chờ pallet được đặt đúng và khu vực trống rồi mới chạy chu trình.
Không mở trùng stack phần cứng/camera đang chạy.

Terminal 1 (giữ mở):

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_pickup_sensors.sh
```

Script build overlay nếu cần, mở bánh xe/IMU/EKF, laser, camera, bộ kiểm tra sàn
và map/AMCL. AMCL dùng nhánh scan tối đa 4 Hz để giảm tải particle cloud;
`/scan_filtered` phục vụ an toàn vẫn giữ nguyên tần số và dấu thời gian.
Chu trình tự mở `pallet_amcl_health.py` để giải mã particle cloud lớn trong tiến
trình riêng rồi nhận bản tóm tắt nhỏ; không giải mã cloud trong executor giám sát
laser/odom. Ngưỡng tuổi dữ liệu nguồn không đổi, bản tóm tắt không làm mới dấu
thời gian pose/particle. Bản sửa này chỉ cần chạy lại terminal chu trình. AMCL không tự nạp pose cũ. Dùng `start_localization:=false` nếu chỉ
cần cảm biến và đã có map/AMCL riêng; không mở hai AMCL cùng lúc.
Không phát lệnh chạy xe hoặc tự HOME. Nếu ESP32 chưa HOME, chỉ thực hiện khi càng
trống và hoàn toàn ngoài pallet:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /lift/home std_msgs/msg/Empty '{}'
ros2 topic echo /lift/status
```

Đợi `homed: true`, `state: IDLE`, `fault: NONE`, càng gần 0; Ctrl+C lệnh echo.
Muốn dừng càng: `ros2 topic pub --once /lift/stop std_msgs/msg/Empty '{}'`.

## Lệnh chạy chu trình

### Tự định vị, về goal nhận diện đã lưu rồi lấy pallet

Terminal phần cứng đang chạy thì giữ nguyên. Terminal chu trình:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_cycle.sh run \
  --auto-localize \
  --revalidate-recognition \
  --pickup-only \
  --usable-depth-cm 35.5
```

Thứ tự: đứng yên kiểm tra AMCL (giữ pose đang tốt, chỉ định vị toàn cục nếu
không đạt trong 12 giây) → Nav2 tới goal nhận diện đã lưu → dừng,
kiểm tra laser/bản đồ → nhận diện xanh mới → căn → checkpoint/đo lại → đưa càng
theo đường thẳng odom → chờ `nang` → nâng 8 cm và dừng. Không khởi động lại
Nav2 sau khi hoàn tất lấy hàng. Không dùng goal pickup bản đồ.

Trước gửi goal, bộ điều phối chờ tối đa 8 giây để lấy lại cửa sổ đứng yên sau
kết nối Nav2 và sau nạp tham số. Vẫn cần ít nhất 0,8 giây dữ liệu đứng yên,
|v| ≤0,003 m/s, |w| ≤0,015 rad/s, càng IDLE và |v_càng| ≤0,002 m/s. Lưu mẫu
theo thời gian thay vì cố định 100 mẫu; thiếu cửa sổ thì đợi, không hủy ngay.
Dữ liệu quá hạn/lỗi cảm biến hoặc goal khác vẫn chặn gửi goal. Khi hết thời
gian chờ, log ghi số mẫu, độ dài cửa sổ, vận tốc lớn nhất và trạng thái càng.

`--revalidate-recognition` cho phép đi tới riêng goal nhận diện mang trạng thái
`needs_visual_revalidation` để nhận diện/căn mới sau khi đến. Không đổi tọa độ,
không ghi goal đó thành đã kiểm chứng, không cho phép dùng goal giao chưa dạy.
Đây là yêu cầu quay lại goal nhận diện đã lưu của người vận hành ngày 03-11-2026.
Không dùng `--from-current-view` hoặc `--from-recognition` trong lệnh này vì hai
cờ đó bỏ qua bước Nav2 chạy về goal nhận diện. Tự định vị không đạt hoặc không
tới goal thì dừng trước bước nhận diện/căn. Vật cản và vùng quét vẫn được kiểm tra.

### Nhận diện/căn ngay tại vị trí hiện tại

Sau khi pallet được mang về và phần cứng/camera đã sẵn sàng, chỉ thử nhận diện/căn:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_cycle.sh run --from-current-view --alignment-only
```

Lệnh này cho xe căn khi cần rồi **kết thúc tại ALIGNED**, không chạy điểm đo lại,
đưa càng hay nâng. Chạy thử bước lấy hàng riêng bên dưới sau khi căn thực tế ổn định.

Terminal 2, sau khi pallet đã sẵn sàng và xe nhìn thấy pallet từ ngoài ≥1,20 m:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_cycle.sh run --from-current-view --pickup-only \
  --auto-localize --usable-depth-cm 35.5
```

Với `--auto-localize`, xe đứng yên để AMCL tìm vị trí trên toàn bản đồ bằng
`/reinitialize_global_localization` và các cập nhật không chuyển động. Không
nạp pose cũ, không cần đặt `2D Pose Estimate`. Sau khi particle cloud tập trung,
chương trình kiểm tra/hiệu chỉnh khớp laser trong vùng ứng viên và đòi pose,
TF, covariance, phân bố particle và scan khớp bản đồ ổn định liên tục **2 giây**.
Theo yêu cầu nới điều kiện ngày 03-11-2026: cửa sổ pose cho phép dao động vị trí
tối đa 5 cm và hướng tối đa 3°, cần ít nhất 5 mẫu mới khác dấu thời gian.
Trước đó là 5 giây / 3 cm / 2° / 8 mẫu. Đây là dao động của pose khi xe đứng
yên, không phải sai số cho phép tại goal. Giữ nguyên độ mới dữ liệu, kiểm tra
laser khớp bản đồ, covariance và điều kiện particle không phân tán. Cửa sổ
RGB-D và xác nhận căn trước đưa càng vẫn có kiểm tra riêng.
Không hội tụ hoặc dữ liệu lỗi thì dừng trước khi mở bộ căn/nhận diện. Môi trường
lặp lại có thể không cho nghiệm rõ khi đứng yên; chương trình không tự chạy xe
để tìm thêm thông tin. Báo cáo ở `auto_localization.json` trong thư mục log lượt chạy.

Sau định vị, đoạn căn/đưa càng vẫn dùng pallet hiện tại và `odom`, không dùng goal
pickup bản đồ. Nếu bỏ `--auto-localize`, chế độ `--from-current-view --pickup-only`
chỉ làm việc cục bộ, không xác nhận xe đã định vị trên bản đồ. Không chạy khi
pallet còn đang ở chỗ khác.

Nếu xe đã đứng ở vùng goal nhận diện và map đúng, dùng `--from-recognition`
thay `--from-current-view` để kiểm tra vị trí trước nhận diện/căn. Muốn xe tự
chạy về goal, dùng lệnh ở mục trên. Goal giao vẫn chưa được dạy lại.

## Dừng và xem đường

Giữ terminal chu trình mở; Ctrl+C dừng xe/càng. Đường mới xuất trên
`/pallet/handling/path` (frame `odom`). Trạng thái có `phase`, mục tiêu, hướng và
tham chiếu `saved_map_goal_used: false`; log nằm trong `data/diagnostics/pallet_cycle_*`.

Đã loại các dịch vụ resume và hạ rồi tiến thêm của phương pháp cũ. Sau lỗi không
tự tiếp tục đường đã dừng; kiểm tra vị trí thực rồi bắt đầu lại từ ngoài pallet.
Điểm pickup bản đồ cũ đã được chuyển vào lịch sử trong `pallet_goals.yaml`.
