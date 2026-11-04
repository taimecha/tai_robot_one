> Historical workflow; do not use these commands for the current pickup controller.

# Một lượt lấy và giao pallet 1

Ngưỡng căn hiện tại được người vận hành nới ngày 03-11: ngang dưới 1,5 cm,
góc dưới 2°. Xe đã ở trong vùng này và còn cách mặt pallet ít nhất 1,20 m
được xác nhận tại chỗ, không cần đạt thêm khoảng dự phòng 1,3225 m của
đường sửa ngang. Quỹ đạo đưa càng vẫn kiểm tra toàn bộ vùng quét, phanh,
vật cản và sự phù hợp với điểm pickup đã dạy trước khi chạy.

Lấy và nâng pallet xanh rồi dừng tại điểm lấy, khi xe đã ở vùng nhận diện:

```bash
bash scripts/open_pallet_cycle.sh run --from-recognition --pickup-only
```

Chế độ này cần goal pickup đã lưu; điểm giao có thể chưa được dạy. Xe vẫn
kiểm tra vị trí vùng nhận diện, định vị, RGB-D, căn chỉnh, quỹ đạo và phản
hồi càng. Khi nâng hành trình 8 cm và xác nhận đứng yên, chương trình kết
thúc tại điểm lấy. Ctrl+C dừng chuỗi điều khiển.

Với pickup có `goal_policy: operator_taught_station_only`, không dùng goal
tương đối khi thiếu TF bản đồ. Điểm tính từ phép đo pallet phải khớp điểm
đã dạy trong 5 mm / 1° trước khi đưa càng. Trước nâng, pose thực tế còn
phải nằm trong 1 cm / 1° của điểm đã dạy và đạt toàn bộ kiểm tra khe/phanh.
Phép đo không phù hợp thì dừng, giữ nguyên tọa độ đã lưu.

Tình trạng kiểm chứng xe thật cuối lượt tối 02-11 (20:33): chưa hoàn tất
đưa càng/nâng/giao. RPP riêng đã đi tới goal căn trong lượt 20:16, nhưng
laser gián đoạn lúc dừng. Các phép đo mới sau chuyển hướng còn cho lệch
trục khoảng 5–7 cm; không chấp nhận ALIGNED giả. Lượt 20:29 dừng vì không
có goal tiến hợp lệ trong khoảng dự phòng còn lại (~1,28 m tới gốc càng).
Điểm dự phòng xa hơn goal nhận diện 35 cm cũng không có đường hoàn chỉnh
trên costmap; đã hủy, đưa xe về nhận diện rồi thử lại. Không tuyên bố chạy
ổn định dài hạn chỉ dựa vào kiểm thử mô phỏng.

Đã xác nhận cuối lượt: xe đứng yên, không có goal Nav2 đang chạy, càng
IDLE/HOME ở 0 m, không lỗi. Báo cáo máy đọc được ở
`data/diagnostics/pallet_recovery_state_20261102.json`. 65 kiểm tra cuối
đạt ở `/tmp/pallet_final_recovery_checks_20261102.log`, cộng các lượt
ROS bám đường/vùng chết đã nêu bên dưới. Các goal lưu có SHA256
`dc339f0a18e175ae4ea4bbf31866d9b8a9677d45adb212916b84d38fe70d80f8`.

Đặt lại pallet xanh tại chỗ lấy cũ, lấy pallet ra khỏi càng và giữ càng ở 0 cm.
Giữ stack phần cứng, camera, AMCL và Nav2 hiện tại chạy; RViz laptop có thể để mở.
Lệnh không tự mở lại serial hay HOME càng. Trước khi chạy, xe phải đứng yên;
laser được đối chiếu với bản đồ. Chỉ hiệu chỉnh định vị khi phép khớp có
nghiệm rõ ràng và được xác nhận bởi một scan mới độc lập.

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_cycle.sh preview
bash scripts/open_pallet_cycle.sh run
```

`preview` chỉ đọc dữ liệu và in các goal; `run` thực hiện đúng một lượt:
về goal nhận diện → nhận diện riêng pallet xanh, chờ YOLO tắt và RGB-D ổn định
→ căn lệch ngang dưới 1 cm và góc dưới 1° với khoảng cách gốc càng tối thiểu
1,2 m → đưa càng theo quỹ đạo odom đã chốt và nâng hành trình 8 cm
→ Nav2 tới goal giao → hạ hành trình về 0 cm → đứng yên, chưa rút càng.

Các goal vẫn lấy từ `ros2_ws/config/pallet_goals.yaml`. Quỹ đạo đưa càng dùng
phép đo pallet hiện tại và kiểm tra tham chiếu goal pickup đã lưu theo giới hạn
hiện có; giữ khoảng cách gốc càng mục tiêu 2 cm, tối thiểu 1 cm đã thống nhất.

Giữ terminal mở, theo dõi xe và dùng Ctrl+C để hủy cả chu trình. Nếu một bước
lỗi (nhận diện, RGB-D, odom, laser, đường đi, nâng/hạ hoặc timeout), chu trình
dừng, không nhảy sang bước sau. Riêng mất dấu khi quay trong bước căn: sau
khi Nav2 hoàn tất và odom xác nhận xe dừng tại goal, cho phép nhận diện lại
pallet xanh tối đa hai lần tại hai lượt đến goal; vẫn chờ YOLO tắt, RGB-D
và bộ căn đạt trước khi căn tiếp. Nếu nhận diện lại thất bại hoặc hết số
lần cho phép, chu trình dừng.
Mỗi lần gọi `run` là một lượt mới;
không dùng để tiếp tục một lượt đã dừng khi càng còn nằm trong pallet.

Khi chuyển điều khiển, chỉ các máy chủ Nav2 và bộ căn được dừng/mở lại.
Phần cứng, cloud camera đã hiệu chỉnh, định vị và odom chạy liên tục. Hai bộ
Collision Monitor không cùng phát lệnh xe. Bộ căn/đưa càng yêu cầu heartbeat
riêng của chu trình; các lệnh con dừng khi tiến trình cha bị kết thúc. Không có
cảm biến tải/tiếp xúc bên dưới, nên COMPLETE/LOWERED xác nhận quỹ đạo và phản
hồi hành trình càng, cần người vận hành quan sát pallet.


Khi càng còn trống, đã hạ và xe đã ở gần pallet sau lượt căn dừng, có thể
nhận diện/căn lại tại chỗ bằng `bash scripts/open_pallet_cycle.sh run --from-current-view`.
Lệnh này vẫn nhận diện mới, kiểm tra dữ liệu, căn đạt và kiểm tra quỹ đạo;
không khẳng định xe đã ở goal nhận diện. Không dùng nếu càng đã vào lỗ.

Định vị trước chu trình và trước giao hàng được kiểm tra bằng khoảng cách
điểm laser tới tường bản đồ. Khớp kém chỉ được sửa khi xe đứng yên, chưa
có bộ căn/đưa càng giữ quyền điều khiển, nghiệm tốt hơn rõ ràng, không có
nghiệm xa tương đương và scan mới xác nhận. Không tìm được nghiệm rõ thì
dừng. Các goal, TF gắn cảm biến và odometry không bị thay đổi bởi bước này.

Nhận diện góc xiên chấp nhận hai chân pallet ở hai cao độ ảnh khác nhau
trong phần đáy hộp YOLO; vẫn cần hai thành phần chân tách biệt và toàn bộ
kiểm tra RGB-D/kích thước. Điều này xử lý góc nhìn xiên mà không hạ ngưỡng
tin cậy YOLO hay bỏ điều kiện màu xanh.

Bước bàn giao sau tắt YOLO giữ cùng một yêu cầu có thời hạn nếu camera
báo một khung lỗi ngay trước coroutine bắt đầu; không gửi lặp lệnh bật
hoặc tự nhận là đã căn. Một phiên nhận diện hết 5 giây được thử mới tối đa
hai lần khi đã xác nhận phiên cũ kết thúc, YOLO tắt/không xử lý, bộ căn
không hoạt động, xe đứng yên và càng trống ở HOME. RPC mất phản hồi, mất
heartbeat hay lỗi khác không được dùng làm lý do gửi lại.

Quay tinh dùng vận tốc góc đo để phanh trước hướng mục tiêu, đợi dừng
sau mỗi nhịp; yêu cầu quay giảm khi còn dưới 3°. Sau khi đã xác minh tới
goal, một khung RGB-D lỗi lập tức đưa lệnh về 0 và dùng bộ đếm mất dữ liệu
liên tục riêng 0,75 giây, không tái sử dụng thời hạn 8 giây của lượt tới
goal. Nếu mất lâu hơn, vẫn yêu cầu nhận diện lại có giới hạn. Trường hợp
bắt đầu quay tại chỗ chỉ được nhận diện lại nếu điểm bắt đầu đã có odom
xác nhận đứng yên, và trước mỗi phiên vẫn kiểm tra đứng yên/cảm biến mới.

Khi khởi động lại ở một điểm căn đã gần pallet, chỉ dùng khoảng dự phòng
lượt đầu 22 cm nếu còn dựng được goal tiến. Nếu các goal này không có đường hợp lệ, thử thêm goal dùng khoảng phanh
thông thường (goal tối thiểu khoảng 1,3225 m ở tốc độ cấu hình), vẫn giữ
kiểm tra phanh và khoảng cách vật lý 1,20 m. Không lùi để khớp mốc 1,42 m.

Dải bám hướng đường Nav2 có dự phòng ±0,08 rad. Trước khi duyệt, kiểm tra
footprint đầy đủ qua cả các góc dự phòng dọc toàn tuyến, gồm các góc trung
gian; ô unknown/vật cản vẫn loại đường. Khi chạy tiếp tục kiểm tra quỹ đạo
phanh thật và hướng ngoài dải đã duyệt vẫn dừng. 31 kiểm tra hình học và
hai lượt ROS (đường chéo đạt, lệch hướng lớn bị chặn) đạt trong
`/tmp/pallet_checked_tracking_reserve_20261102.log`.

Đường căn còn phải lọt toàn bộ polygon StopZone đang chạy, không chỉ thân
càng cộng đệm 3 cm. Polygon giữ đệm tối thiểu 18 cm như Collision Monitor;
điểm lidar thật nằm trong vùng dừng không được coi là đường đi hợp lệ.
Sửa này ngăn duyệt đường đi sát bàn/ghế rồi bị CM dừng giữa đường.
32 kiểm tra hình học đạt: `/tmp/pallet_route_stopzone_consistency_20261102.log`.

Đường thẳng gửi cho RPP nay chỉ gồm các điểm tịnh tiến và hướng cuối; các
động tác quay đầu/cuối vẫn được kiểm tra footprint đầy đủ trước đó. Không
gửi một dãy điểm trùng XY để mô tả quay: xe skid-steer trôi vài cm làm cusp
quay cũ ở phía sau xe và bộ bám đuổi lại cusp. Mô phỏng ROS có trôi tiến
4 mm/s khi quay đạt ALIGNED 1 cm/1° trong
`/tmp/pallet_unpinned_cusp_ros_20261102.log`.

FootprintGrid duyệt ô lethal/unknown bằng NumPy trên buffer ROS, tránh
duyệt từng ô trống bằng Python ở mỗi pose. Không đổi tiêu chuẩn vật cản.
Một tuyến 43 pose có đủ dải quay kiểm tra khoảng 40 ms trong thử tại Pi.

Bộ bám riêng `/pallet_nav` dùng RPP chuẩn để đi đoạn chéo ngắn.
SafeRotationRPP của Nav2 bản đồ giữ nguyên; logic hồi phục đoạn cuối 15 cm
của nó không áp dụng vào bước tới trục pallet. Bộ căn vẫn kiểm tra đường,
chặn lùi, hủy đúng action, phanh theo vận tốc đo và sửa góc bằng RGB-D.
Ba kiểm tra ROS (trôi quay, vọt tốc độ, hủy đang chạy) đạt trong
`/tmp/pallet_dedicated_rpp_ros_20261102.log`.

RPP ở 20 Hz với max_angular_accel=0,3 chỉ xin ~0,015 rad/s từ đứng yên,
thấp hơn vùng chết bánh xe thật. Bộ bám riêng nay dùng max_angular_accel=2
để tạo yêu cầu đủ khởi động; tầng căn phía ngoài vẫn hạn gia tốc 0,40 và
phanh đầy đủ theo tốc độ thực. Mô phỏng vùng chết 0,06 rad/s đạt ALIGNED
trong `/tmp/pallet_rpp_real_deadband_ros_20261102.log`. Lượt xe thật 20:16
đã tới goal căn và dừng, sau đó dừng chu trình vì laser quá hạn; chưa đưa
càng/nâng trong lượt đó.

Log mỗi lượt: `data/diagnostics/pallet_cycle_YYYYmmdd_HHMMSS/`.

Sau chạy thật 18:38/18:42: lượt đầu giữ được khoảng 1,42–1,43 m; lượt sửa
ngang có thể mất dấu thêm một lần. Cho phép hai lần nhận diện lại sau xác
nhận dừng, giới hạn hiệu chỉnh vẫn hữu hạn; không thử lại lỗi laser/odom/
vật cản. Bộ kiểm thử xác nhận mất dấu lần thứ ba kết thúc chu trình.

Lượt 18:42 dừng vì chuyển xoay→tiến có độ vọt yaw lớn hơn 20%: đo được
v≈0,093 m/s, w≈0,172 rad/s, quãng phanh 19,94 cm. Lệnh nay dự tính yaw
cao hơn yêu cầu tối đa 2 lần (tiến vẫn 1,2 lần), đồng thời mỗi tick chặn
theo tổng vận tốc lớn nhất giữa tốc độ đo còn lại và tốc độ yêu cầu dự tính
trên từng trục. Khi trục cũ chưa chậm, giảm cùng tỷ lệ cả hai lệnh, không
giả định đã phanh ngay. Ngân sách lệnh 16 cm và dừng đo thật 18 cm giữ nguyên.
52 kiểm tra thuần và hai mô phỏng Nav2 đạt: tiến vọt 18%/yaw vọt 80% vẫn
ALIGNED theo 1 cm/1°, yaw vượt thật 0,300 rad/s dừng ở 20,95 cm. Log
`/tmp/pallet_measured_transition_{unit,ros}_20261102.log`.

Cập nhật sau lượt 18:24 ngày 02-11: yêu cầu mới là ngang <1 cm và góc <1°.
Áp dụng vào config/default bộ căn, config/default đưa càng, kiểm tra phép đo
mới ngay trước lập quỹ đạo và hiển thị web. Vùng gần trục pháp tuyến 3 cm
cho phép quay tinh hướng vào tâm lỗ; đó không phải ngưỡng đưa càng. Sau
quay, Y tâm lỗ trong hệ xe phải <1 cm và góc hai mặt <1°. Quỹ đạo đưa càng
còn kiểm tra toàn bộ hình học: khi mũi càng vào lỗ, lệch trục <1 cm, góc
<1° và hai càng không chạm chân pallet. Chỉ ALIGNED cũ không đủ đưa càng.
Nếu đường thẳng tới tâm cuối vượt 1°, có thể dịch điểm cuối vài mm trong
lỗ; vẫn phải đạt toàn bộ các giới hạn hình học trên mỗi pose.

Nguyên nhân NO_SAFE_GOAL sau nhận diện lại: lần tiếp cận đầu kết thúc cách
mặt pallet ~1,30 m, ít hơn khoảng dành cho quỹ đạo sửa ngang (~1,3225 m).
Lượt đầu nay chọn goal có dự phòng 22 cm ngoài mốc 1,20 m (~1,42 m trở lên).
Sau đến goal và nhận diện lại, giữ bộ đếm hiệu chỉnh và dùng ngân sách phanh
thông thường cho lượt sửa; không reset thành lượt đầu và không tiến dưới
1,20 m. Lệnh mới hoàn toàn vẫn bắt đầu với dự phòng lượt đầu. Không sửa goal
nhận diện, pickup hoặc dropoff đã lưu.

Nav2 có thể muốn xoay đuổi một endpoint đã vượt vài mm dù đã tới trục:
nếu cách goal ≤3 cm, hướng lệch pháp tuyến ≤5°, chuẩn bị dừng/xoay (v≤0,005),
đồng thời nằm trong vùng gần trục 3 cm và khoảng cách ≥1,22 m, hủy FollowPath theo đúng
chuỗi xác nhận hủy/đứng yên. Sau đó vẫn bắt buộc ảnh mới và ngưỡng cuối
1 cm/1°. Kiểm tra quỹ đạo phanh, laser/odom, footprint giữ nguyên.

Kiểm chứng trước chạy thật: 80 kiểm tra hình học/chu trình/đưa càng đạt,
13 kiểm tra lọc góc/web đạt; mô phỏng Nav2 căn bình thường và vận tốc bám
vọt 18% đạt ALIGNED theo 1 cm/1°. Kiểm tra ROS đưa càng từ trạng thái
ALIGNED cũ nhưng phép đo mới vượt ngưỡng bị chặn; mô phỏng đưa càng/nâng
qua Collision Monitor hoàn tất. Log `/tmp/pallet_1cm_1deg_*_20261102.log`.

Sửa sau lượt 18:14 ngày 02-11: planner tìm được đường (error_code=0),
nhưng kiểm tra footprint loại ba candidate bởi cùng một ô unknown (-1)
tại odom (-0,075; -0,175); candidate còn lại bị giới hạn hướng. Không có
bằng chứng trong log rằng ô đó là vật cản lethal. Costmap dựng lại tại xe
đứng yên cho 24/24 snapshot đường tiến chéo hợp lệ. Chưa xác định vì sao
đúng snapshot cũ còn ô unknown; không đổi unknown thành free.

Bộ căn nay giữ xe đứng yên, chờ tối đa 2 giây cho từng candidate có đường
bị chặn bởi unknown, chỉ tính lại khi stamp costmap tăng. Cả đường tiến
chéo và đường lattice vẫn được kiểm tra toàn bộ footprint/padding/khoảng
cách trước khi gửi FollowPath. Dữ liệu không bổ sung đủ thì loại candidate
như cũ; vật cản lethal, đường ngang/lùi, thiếu khoảng cách vẫn bị chặn.
Log giữ lý do riêng của cả đường trực tiếp và lattice để không che khuất
nguyên nhân đường trực tiếp bị loại. Không đổi goal hoặc footprint.

Kiểm chứng: 45 kiểm tra điều kiện/hình học/chu trình đạt và hai mô phỏng
Nav2 thật/base giả đạt: unknown tạm thời đợi snapshot mới rồi PREVIEW_READY,
unknown kéo dài luôn NO_SAFE_GOAL, không gửi vận tốc. Log
`/tmp/pallet_unknown_retry_{unit,ros}_20261102.log`.
Thử trên Pi: chỉ mở planner/costmap riêng, tái dựng mục tiêu từ phép đo đã
chốt trong log 18:14 (xe vẫn nguyên odom trong 5 mm/0,01 rad), ComputePath
thật và bộ kiểm tra đường hiện tại cho đường hợp lệ 49 pose. Không gửi
FollowPath hay lệnh nâng. Kết quả
`data/diagnostics/pallet_unknown_grid_route_preview_20261102.json`.
Planner/costmap dùng kiểm tra đã được đóng, phần cứng/camera/AMCL giữ chạy.

Sửa sau lượt 18:07 ngày 02-11: Nav2 yêu cầu lùi nhẹ sau khi đã tới trục
pallet; nhánh cũ chặn lùi bằng REACQUIRE_REQUIRED tức thì, chưa có xác nhận
dừng tại goal nên chu trình phải dừng. Nay chỉ khi sai lệch khỏi trục dưới
1,5 cm theo snapshot và gốc càng còn cách mặt pallet ≥1,22 m, chặn lệnh
lùi rồi chuyển NAV_CANCELING. Phát lệnh 0, hủy đúng FollowPath đang chạy,
đợi cả phản hồi hủy lẫn kết quả action kết thúc (CANCELED/SUCCEEDED).
Trong lúc này mọi lệnh Nav2 mới đều bị bỏ qua; chờ quá 3 giây hoặc hủy
thất bại thì BLOCKED. Laser/odom/Collision Monitor vẫn bắt buộc hợp lệ.

Sau xác nhận hủy mới vào NAV_STOPPING, đợi odom xác nhận đứng yên liên tục
rồi CHECKING và lấy RGB-D mới để sửa góc. Mất dấu thì chu trình dùng nhánh
nhận diện lại duy nhất đã có; không gán ALIGNED hay cho phép đưa càng chỉ
từ snapshot. Lùi khi còn lệch trục hoặc thiếu khoảng cách vẫn BLOCKED.
Mô phỏng Nav2/Collision Monitor thật và đế giả đã hủy đường theo nhánh này,
sửa góc bằng ảnh mới và đạt ALIGNED; không phát vận tốc lùi.
Kiểm chứng: 51 kiểm tra điều kiện/hình học/chu trình đạt, gồm hủy bị từ
chối, chờ cả phản hồi hủy và kết quả terminal, lệnh lùi ngoài trục bị chặn,
hết thời gian hủy không được chuyển sang xác nhận tới goal. Ba kiểm tra
ROS đạt: hủy tại trục rồi ALIGNED, nút dừng hủy đường/bỏ lệnh trễ và mất
laser dừng. Log `/tmp/pallet_axis_handoff_*_20261102.log`. Không gửi
chuyển động hoặc nâng trên xe thật trong lượt sửa này.

Sửa sau lượt 18:01 ngày 02-11: bước delivery return kiểm tra danh sách
publisher status Nav2 đúng một lần ngay sau lifecycle ACTIVE; kiểm tra
hiện tại trên Pi chỉ thấy một bt_navigator. Không còn số lượng/GID trong
log lỗi cũ để phân biệt graph chưa khám phá đủ với endpoint cũ chưa hết hạn.
Thay kiểm tra tức thời bằng chờ tối đa 10 giây: đúng một endpoint global
`/bt_navigator`, action server sẵn sàng, cùng GID ổn định liên tục 1 giây.
Thiếu server, trùng server, sai namespace hoặc thay GID đều đặt lại cửa sổ;
hết hạn dừng và in danh sách endpoint/action_ready. Kiểm tra lại cùng GID
ngay trước gửi goal sau nạp tham số; không chấp nhận server đổi giữa chừng.
Các điều kiện xe đứng yên, cảm biến/càng, lifecycle và goal khác giữ nguyên.
Kiểm chứng: 23 kiểm tra điều kiện/chu trình đạt; kiểm thử DDS domain 227
chờ action server xuất hiện muộn và từ chối hai server thật trùng tên đạt,
không gửi goal. Trên Pi, client mới xác nhận đúng một server ACTIVE trong
2,88 giây, không gửi chuyển động. Kết quả:
`data/diagnostics/delivery_discovery_stationary_20261102_180550.json`.

Sửa sau lượt 17:52 ngày 02-11: Nav2 báo tới goal sau ~9 giây nhưng kẹt
`NAV_STOPPING` đến timeout 60 giây. Bản tin RGB-D invalid liên tục gọi
`invalidate_target()`, xóa bộ đếm đứng yên dùng chung với bộ căn bằng ảnh.
Tách bộ đếm dừng theo odom khỏi bộ đếm xác nhận ảnh. Xác nhận dừng vẫn cần
odom mới và tốc độ thấp liên tục 0,5 giây; chuyển sang CHECKING rồi bắt buộc
phép đo mới. Chưa dừng được trong 5 giây sau kết quả Nav2 thì BLOCKED,
không dùng timeout đi đường để chờ ảnh. Không có ảnh mới sau dừng thì yêu
cầu nhận diện lại, không được coi là ALIGNED hay đưa càng.

Chu trình xử lý đúng một lần nhận diện lại này khi có trạng thái mới
`REACQUIRE_REQUIRED`, `stationary_arrival_confirmed=true`, Nav2 đã xong,
bộ căn inactive và khoảng cách dự kiến còn ≥1,2 m. Kiểm tra lại xe đứng yên,
cảm biến/càng trống, nhận diện phiên mới đúng pallet xanh, chờ YOLO tắt và
RGB-D ổn định, rồi mới xin căn lại. Các lỗi BLOCKED/TIMEOUT/laser/odom
không được đi vào nhánh này; mất dấu lần hai hoặc nhận diện thất bại dừng.

Kiểm chứng: kiểm tra điều kiện/hình học/chu trình và hai kiểm tra ROS
Nav2/Collision Monitor với base giả đạt, gồm phát RGB-D invalid liên tục
qua lúc đến goal và đường căn bình thường vẫn ALIGNED. Trên Pi thật chỉ
nhận diện tại vị trí đang dừng: pallet xanh TRACKING, YOLO tự tắt, cửa sổ
RGB-D 75/75 phép đo hợp lệ, 15,11 Hz, ngắt lớn nhất 0,139 giây. Xe đứng yên,
càng IDLE 0 cm; không gửi goal chuyển động/nâng. Dữ liệu:
`data/diagnostics/arrival_reacquire_stationary_20261102_175924/`.

Sửa sau lượt 17:41 ngày 02-11: xe dừng khi quãng phanh tính từ vận tốc đo
vượt 18 cm trong lúc vừa tiến vừa xoay. Trace cho thấy vận tốc tiến thực tế
cao hơn lệnh khoảng 10–15%; ngân sách lệnh cũ chưa dự tính độ vọt này.
Bộ giới hạn nay dự tính vận tốc cả hai trục cao hơn lệnh 20%, rồi giới hạn
tổng quãng phanh còn 16 cm, giảm đồng thời v/w để giữ độ cong. Áp dụng cả
lệnh Nav2 lẫn xoay sửa góc cuối. Ví dụ lệnh gần lúc dừng 0,0983 m/s và
0,1007 rad/s nay còn khoảng 0,0819 m/s và 0,0839 rad/s; tiến thẳng vẫn tối
đa 0,10 m/s. Kiểm tra vận tốc đo thật vẫn dừng nếu vượt 18 cm; độ dự phòng
20% không phải bảo đảm vật lý cho mọi độ vọt. Lý do dừng in quãng phanh
đến 0,01 cm để tránh hiển thị hai vế cùng làm tròn thành 18 cm.

Kiểm chứng: 40 kiểm tra thuần đạt, gồm trường hợp vừa tiến vừa xoay sát
ngưỡng trong log và kiểm tra giữ độ cong/dự phòng 20%. Bốn kịch bản ROS
với Nav2/Collision Monitor thật, base giả đã đạt: bám cả tiến/xoay vọt 18%,
xoay riêng vọt 18%, mất laser phải dừng, và vượt tốc thật phải dừng
(w=0,279 rad/s, quãng phanh 18,96 cm). Khi chạy chung dưới tải, kiểm tra
vượt tốc có lần dừng sớm do odom không mới; chạy riêng xác nhận đúng lý do
vượt tốc, không thay hạn odom. Log `/tmp/pallet_tracking_margin_*_20261102.log`.
Nav2 toàn cục trên Pi đã mở lại ở trạng thái chờ, không gửi goal; càng
IDLE/homed 0 cm. Chưa thử chuyển động thật bằng công cụ.

Khôi phục sau lỗi laser lúc 16:03 ngày 02-11: driver `/scan` vẫn chạy,
nhưng tiến trình `scan_self_filter` mở riêng trước đó đã thoát với
`ExternalShutdownException`, không còn publisher `/scan_filtered`.
AMCL mất scan nên TF map→odom hết hạn và RobotModel biến mất trên RViz.
Chưa xác định được nguồn gửi shutdown; không phải lỗi ngưỡng 400 ms.

Phiên phần cứng cũ hiện dùng dịch vụ tạm systemd user
`tai-pallet-scan-filter-recovery.service`, chỉ chạy bộ lọc với đúng hình học
cũ và `max_scan_age_s:=0.40`, tự khởi động lại khi lỗi (tối đa 3 lần/30 giây).
Giữ dịch vụ này khi chạy chu trình hiện tại. Khi đóng TOÀN BỘ stack để mở
một phiên phần cứng mới, cần dừng cả dịch vụ tạm để tránh mở hai bộ lọc:

```bash
systemctl --user stop tai-pallet-scan-filter-recovery.service
```

Launch `scan_self_filter.launch.py` đã thêm respawn sau 1 giây cho các phiên
mới; không khởi động lại driver, đổi stamp hay cho phép tiếp tục chu trình đã
dừng. Kiểm thử domain 229: kết thúc tiến trình lọc, launch mở lại, dữ liệu
quá hạn vẫn bị loại, dữ liệu mới giữ nguyên stamp/ranges, đạt.

Đọc xe thật sau cập nhật, không gửi chuyển động: 30 giây, 1.154 lần kiểm tra,
không mất dữ liệu; tuổi laser lớn nhất 0,289 s, odom 0,063 s, TF hợp lệ,
đúng một publisher scan_filtered, càng IDLE 0 cm. Dữ liệu:
`data/diagnostics/laser_recovery_live_20261102.json`.

Sau lần tắt/bật nguồn chiều 02-11, đã xác nhận điểm sàn giả trong camera:
ảnh RGB ghép với cloud lệch 35 ms, sàn trống có điểm cao 6,6 cm và các mẫu
muộn hơn còn cao hơn. Người vận hành xác nhận không chạm camera; không thấy
khung TF camera trùng. Chưa xác định chắc nguồn thay đổi giữa hai phiên.

Launch camera nay dùng hồ sơ bù cao độ
`astra_pro_nav_floor_2026_11_02_restart.yaml`. Fit 10 frame và kiểm tra 10
frame khác trên vùng sàn đã đối chiếu RGB (loại pallet/kệ/ghế); 86.220 điểm
kiểm tra không vượt ngưỡng 6 cm, percentile 99 sai số 0,92 cm. Chỉ đổi cao
độ, giữ nguyên X/Y, hiệu chỉnh inverse-depth, TF, cửa sổ 6–75 cm và các goal.
Thử nguồn mới 30 frame: 258.660 điểm sàn không vượt 6 cm; điểm pallet xanh
vẫn được giữ. Bốn kiểm tra hình học cao độ đạt. Đây là kiểm chứng trong
phiên hiện tại, chưa chứng minh sai số không thay đổi sau mọi lần bật nguồn.

Nguồn cloud phiên này được nạp lại riêng để không ngắt driver/AMCL/càng.
Supervisor `data/runtime/nav_cloud_replacement_20261102.py` theo dõi PID và
start-time của launch phần cứng hiện tại; tự dừng cloud khi launch đó kết
thúc để không còn nguồn riêng ở lần mở toàn bộ tiếp theo. Log:
`/tmp/pallet_cloud_restart_height_20261102.log`. Không gửi chuyển động.
Dữ liệu RGB/cloud trước/sau và phép kiểm chứng nằm trong
`data/diagnostics/obstacle_restart_20261102/`.

Vùng màu loang lớn trong RViz còn gồm inflation quanh vật cản thật, bán kính
cấu hình 0,88 m; không được coi toàn bộ màu đó là ô vật cản trực tiếp. Khi
xe/goal đứng yên, đã làm mới hai costmap động để loại dấu sàn cũ; bản đồ
tĩnh và vùng đệm an toàn không thay đổi.
Theo dõi sau làm mới 35 giây: 526 frame, không điểm sàn nào vượt 6 cm,
cao độ sàn lớn nhất 4,46 cm, đầu/cuối cửa sổ không tăng đáng kể; đúng một
nguồn cloud, xe đứng yên, càng IDLE 0 cm. Kết quả chi tiết:
`obstacle_restart_20261102/after_clear_monitor.json`.

Điểm sàn tái xuất hiện sau khi mở lại camera dù đúng hồ sơ mới: 114 điểm
vượt 6 cm trong bộ dữ liệu đối chiếu RGB. Phép thử 35 giây trước đó chỉ
chứng minh phiên ngắn đó; bù phẳng cố định chưa đủ cho sai số giữa các phiên.

Đã thêm `floor_surface_model.py`, bật bằng
`floor_surface_tracking_enabled` trong hồ sơ camera hiện tại. Bộ ước lượng
chỉ nhận vùng gần mặt đất rộng, có hỗ trợ hai bên và gần/xa, với mốc cao độ
giới hạn ±3 cm tại X=1,7 m và độ nghiêng dư tối đa 0,10. Fit robust cần ít
nhất 75% ô XY hỗ trợ; một mặt pallet/vật thấp nâng cao hoặc vùng quá nhỏ
không được dùng làm sàn. Cần ít nhất 8 phép đo nhất quán trong >=3,5 giây,
odom mới <=200 ms và xe đứng yên trước khi cập nhật. Đang chạy, mất odom,
thiếu sàn hoặc không đủ độ ổn định: giữ mô hình cuối đã xác nhận.

Mô hình chỉ bù Z, giữ X/Y và điểm không hợp lệ, không xóa vật cản theo vùng
bản đồ. Mất phép đo hợp lệ không tạo ra tia tự do. Khi vừa mở hoặc đổi
CameraInfo, chưa có mô hình được xác nhận thì chưa phát cloud Nav; timeout
camera hiện có giữ xe dừng. Các ngưỡng 6–75 cm, footprint, TF và goal giữ
nguyên. Chu trình/preview nay chờ trạng thái mới qualified=true trên
`/camera/floor_height/status` trước khi bắt đầu.

Kiểm chứng: 26 kiểm tra điều kiện mặt sàn, vật thấp/pallet, đóng băng khi
chạy/mất odom và điều kiện chu trình đạt. Phát lại ba bộ cloud của các phiên
khác nhau: không còn điểm sàn giả trong vùng RGB kiểm chứng, pallet xanh
giữ cao độ trung vị 11,5–12,2 cm, X/Y không đổi. Sau khi người vận hành đi
ra, thử xe thật đứng yên 35 giây: 525 frame, 4.526.549 điểm sàn không vượt
6 cm, cao độ sàn tối đa 1,55 cm, điểm pallet vẫn còn. Lượt trước có người
đi qua được giữ nguyên trong log nhưng không dùng làm phép thử sàn trống.
Log và dữ liệu: `data/diagnostics/obstacle_recurrence_20261102/`.
Thử đóng/mở lại tiến trình cloud, giữ nguyên driver và định vị, không fit
lại hồ sơ cố định, không xóa costmap thủ công: 45 giây, 676 frame,
5.828.465 điểm sàn không vượt 6 cm, cao độ sàn tối đa 1,67 cm. Mỗi frame
vẫn có ít nhất 523 điểm trong vùng pallet cao hơn 6 cm. Preview chu trình
với điều kiện qualified mới đạt; xe đứng yên, càng 0 cm. Kết quả:
`tracking_after_restart.json`. Đây là kiểm tra nguồn cloud khởi động lại,
không phải thử tắt/bật lại toàn bộ phần cứng bằng công cụ.

Lượt 17:30 dừng ở nhận diện, chưa bắt đầu căn. Thử riêng nhận diện khi xe
đứng yên tái hiện vòng zoom báo không tách được hai chân trước; góc đo trong
lượt cũ khoảng 17°. Tìm kiếm nay giữ nguyên view đã cho seed hợp lệ để xác
nhận trên các ảnh mới, zoom thất bại trả về toàn ảnh thay vì tiếp tục phóng
to một mask thiếu chân. Các ngưỡng seed, hình học, tuổi dữ liệu và 5 giây
không thay đổi. 27 kiểm tra tìm kiếm/chu trình đạt; chưa khẳng định nhận
diện xe thật sau sửa ở tư thế xiên cho tới lượt thử tiếp theo.

Theo đề nghị của người vận hành, Nav2 được mở lại riêng ở trạng thái chờ,
không gửi goal bằng công cụ. Người vận hành có thể chạy
`bash scripts/open_pallet_delivery.sh return` để về đúng goal nhận diện,
đợi ARRIVED rồi chạy chu trình mới `run --from-recognition`. Hoặc dùng
`open_pallet_cycle.sh run` thông thường để tự thực hiện cả bước về nhận diện.
Các lượt mới ghi thêm `recognition.jsonl` (phase, số seed/mẫu đo, lý do từ
chối và thời gian suy luận) để không mất nguyên nhân timeout khi dọn node.

Nếu lượt trước đã tới vùng nhận diện, đã dừng trước khi đưa càng và càng còn
trống ở 0 cm, có thể bắt đầu một lượt mới từ đó:

```bash
bash scripts/open_pallet_cycle.sh run --from-recognition
```

Lệnh kiểm tra vị trí trong 25 cm, 20° so với goal nhận diện, rồi nhận diện và
căn mới hoàn toàn. Không tiếp tục một quỹ đạo cũ hay bỏ qua các điều kiện đưa
càng. Xe vẫn phải đạt ngang <2 cm, góc <2° và khoảng cách ≥1,20 m sau bước căn.

Sau lượt 15:50: hai lần kiểm tra vùng nhận diện nay chờ tối đa 5 giây để
nhận đủ odom/TF mới sau khám phá dịch vụ và phân bổ CPU. Giữ nguyên ngưỡng
vị trí, hướng, vận tốc đứng yên và tuổi dữ liệu. Lỗi phân biệt rõ ngoài vùng
với chưa đủ mẫu đứng yên; `recognition_station.json` trong log lưu khoảng
cách, góc, pose và kết quả đứng yên khi kiểm tra lần hai. Đọc lại xe thật:
cách goal 5,4 cm, lệch hướng 14,4°, đứng yên; `preview --from-recognition` đạt.
Chưa tái hiện được chính xác điều kiện thoáng qua gây lỗi trong lượt cũ.

RViz đầy đủ dùng `ros2_ws/config/nav_robot_camera.rviz` (đã đồng bộ laptop).
Robot Footprint (live) lấy `/rviz/robot_footprint` từ node chỉ hiển thị
`rviz_footprint_display`. Node nhận footprint costmap Nav2 hoặc pallet_nav,
đổi hình học về base_footprint bằng TF đúng thời điểm đo rồi cho Marker bám
TF hiện tại như RobotModel. Marker giữ nguyên footprint có padding/tải,
hết hạn tối đa 0,8 giây khi nguồn dừng; không giữ hình ở hướng cũ trên RViz.
Các topic footprint dùng cho planner/Collision Monitor giữ nguyên.

Node hiển thị đi cùng phần cứng trong lần mở `nav_real.launch.py` mới,
không mở trùng khi `reuse_sensors:=true`. Phiên phần cứng hiện tại được mở
trước cập nhật nên có node hiển thị riêng (log
`/tmp/pallet_rviz_footprint_20261102.log`); dừng cùng stack khi đóng toàn bộ.
Nav2 đã được mở lại riêng để cập nhật costmap trên RViz, không gửi goal;
AMCL/driver/odom/càng không khởi động lại. 16 kiểm tra chu trình và hiển thị
đạt; kiểm tra thực nhận footprint 10 Hz, hình học đúng, càng 0 cm, xe đứng yên.

Sau lượt 15:38, theo yêu cầu nới dung sai: lệnh tiếp tục dùng ngân sách phanh
16 cm; vận tốc đo được phép dùng đủ 18 cm của polygon (phần dự phòng 2 cm dành
cho sai số bám/độ vọt). Vượt 18 cm vẫn dừng. Tốc độ xoay căn tối đa hạ từ
0,24 xuống 0,22 rad/s. Dung sai hướng lúc bắt đầu từ vùng nhận diện là 20°;
RGB-D vẫn phải chốt được pallet mới và điều kiện căn đạt cuối vẫn <2 cm/<2°.

Lượt lỗi cũ chưa ghi v/w ngay lúc dừng nên chưa xác định được giá trị vượt
ngưỡng. Bản mới in cả v, w, quãng phanh đo và giới hạn vào lý do dừng/log;
`motion.jsonl` lưu trạng thái căn và odometry 5 Hz. Kiểm tra mô phỏng với
Nav2/Collision Monitor thật, base giả: độ vọt yaw 18% vẫn ALIGNED; trường hợp
vượt vùng 18 cm dừng BLOCKED, in đủ giá trị đo. 50 kiểm tra thuần và hai kiểm
tra ROS này đều đạt. Xe thật chỉ được đọc dữ liệu trong lượt sửa này: đứng
yên, càng 0 cm; preview từ vùng nhận diện hiện tại đạt.

Cập nhật sau lượt 15:21: giới hạn tuổi laser hiện là **400 ms**, đồng bộ bộ
lọc `/scan_filtered`, MotionGuard, Collision Monitor, bộ đưa càng và cửa sổ
ổn định trước chạy. LiDAR thực quét khoảng 7 Hz: một vòng mất ~140 ms và tuổi
tia đầu gần 280 ms trước vòng tiếp theo; 300 ms không đủ dự phòng độ trễ xử
lý. Stamp, góc, ranges và hình học quét giữ nguyên. Hết 400 ms vẫn dừng ngay;
odom 200 ms, RGB-D 250 ms, các goal và ngưỡng căn không thay đổi.

Vùng dừng Collision Monitor giữ nguyên, tối thiểu 18 cm ngoài thân/càng.
Giữ dự phòng 2 cm; lệnh Nav2 được giảm đồng thời v/w khi tổng quãng dịch thân
và đầu càng qua 400 ms dữ liệu cũ + 100 ms phản ứng + phanh bảo thủ vượt
16 cm. Giảm cùng tỷ lệ giữ độ cong đường đã kiểm tra. Tiến thẳng 0,10 m/s
và xoay tại chỗ 0,22 rad/s vẫn nằm trong giới hạn; vận tốc đo vượt vùng bảo
vệ sẽ dừng. Mã kiểm tra cũng xác minh polygon chứa đủ dự phòng đã tính.

Kiểm chứng bản 400 ms: 49 kiểm tra thuần, kiểm tra ROS của bộ lọc giữ stamp,
2 kiểm tra đưa càng/nâng và mất laser/vật cản, 2 kiểm tra Nav2 căn và hủy khi
mất laser đều đạt. Thử trên Pi với camera, nhận diện/RGB-D, Nav2 costmap,
MotionGuard và Collision Monitor thật, chỉ phát vận tốc **0**, không mở
handler/nâng và không gửi FollowPath: 60 giây, 1.201 lần kiểm tra, không lần
mất điều kiện dữ liệu; tuổi laser lớn nhất 309 ms. Xe đứng yên, càng IDLE
0 cm. Kết quả: `data/diagnostics/pallet_stationary_timing_20261102_153411/`.

Trong phiên cập nhật hiện tại, chỉ bộ lọc laser được nạp lại riêng để giữ
phần cứng/odom/AMCL/camera liên tục. Log `/tmp/pallet_scan_filter_400ms_20261102.log`.
Lần mở toàn bộ stack mới dùng launch đã cập nhật; cần dừng cả bộ lọc riêng
này cùng stack hiện tại trước khi mở mới để tránh hai nguồn `/scan_filtered`.
`run`/`preview` xác minh tham số bộ lọc là 0,40 s trước khi làm việc.

Sửa sau lượt 15:12: YOLO đã thực hiện yêu cầu nhưng FastDDS báo không gửi được
phản hồi dịch vụ. Client nay tồn tại từ lúc mở chu trình; trước bật YOLO chờ
camera, trạng thái và dịch vụ sẵn sàng. Gửi đúng một yêu cầu, xác nhận bằng
`session == phiên trước + 1` cùng `started_ros_s >= lúc gửi` trên heartbeat mới.
Phản hồi RPC thành công một mình hoặc trạng thái TRACKING cũ không đủ để đi
tiếp. Yêu cầu bị từ chối hay không có phiên mới vẫn dừng; không tự gửi lại.
Sau đó vẫn chờ YOLO tắt, đúng phiên và toàn bộ cửa sổ RGB-D đạt như trước.

Kiểm chứng: 18 kiểm tra điều kiện/thứ tự/CPU đạt; 3 kiểm tra ROS domain 228
đạt gồm mất phản hồi, trạng thái TRACKING cũ và từ chối dịch vụ. Thử riêng
nhận diện trên Pi thật đạt TRACKING, YOLO tắt/hết xử lý, RGB-D 15,03 Hz hợp lệ
100%, ngắt lớn nhất 0,109 giây và stable=true. Xe đứng yên, càng 0 cm; không mở
bộ căn hay gửi chuyển động. Log và kết quả:
`data/diagnostics/cycle_ack_stationary_probe_20261102/`.

Sửa sau lượt 14:56 ngày 02-11: chu trình dừng do laser quá 300 ms trong bước
căn. Tác vụ ảnh và planner/controller dùng ba lõi CPU; laser/đế xe/TF, bộ căn
và Collision Monitor vẫn truy cập được lõi còn lại. Ái lực CPU của các tiến
trình có sẵn được khôi phục khi kết thúc. Trước căn, chờ laser/odom liên tục
5 giây và trạng thái bộ căn/RGB-D sẵn sàng. Giữ nguyên stamp thực và mọi hạn
tuổi dữ liệu; không tự khởi động lại sau lỗi.

Kiểm tra lại: 17 kiểm tra điều kiện/thứ tự/lỗi/CPU đạt; chuyển Nav2 → bộ căn
→ Nav2 trong domain 232 với cảm biến giả đạt, gồm kiểm tra mask CPU của hai
server staging. Trên Pi thật, sau 10 giây khám phá/ổn định, đo 30 giây dưới
tải RGB-D và Nav2: 3.001 lần kiểm tra, không lần laser quá 300 ms, tuổi lớn
nhất 0,289 giây. Dữ liệu: `cycle_laser_steady_probe_20261102.json`. Kiểm tra
vùng nhận diện hiện tại đạt, càng MCU IDLE tại 0 cm. Chưa gửi chuyển động thật.

Kiểm chứng ngày 2026-11-02: 14 kiểm tra thứ tự/điều kiện/lỗi và tiến trình cha;
2 kiểm tra ROS với base/càng giả và Collision Monitor thật về mất heartbeat;
1 kiểm tra Nav2 thật → bộ căn → Nav2 trong domain 232 với cảm biến giả, xác
nhận không mở lại driver/AMCL/camera và nhận đủ trạng thái bộ căn. Các kiểm tra
đều đạt. `preview` trên xe thật cũng đạt với càng 0 cm, odom/laser/TF mới.
Chưa chạy chu trình thật tự động bằng công cụ; người vận hành tự chạy.
