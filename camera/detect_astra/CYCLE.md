# Giới hạn góc căn ngoài pallet: 30°

Giới hạn staging và checkpoint đều là 30°. Kiểm tra cả góc quay từ hướng xe
và góc đường tiến so với pháp tuyến pallet; giữ kiểm tra toàn bộ thân/càng,
quỹ đạo phanh và giới hạn riêng khi đưa càng trong khe.
Điểm đích nằm trên pháp tuyến qua tâm lỗ: goal = opening -
(fork_root + checkpoint_gap) * hướng pháp tuyến. Góc tiến dùng atan2 của
vector từ pose hiện tại tới goal; quãng tiến là độ dài vector đó.
Terminal in hình học đường tiếp cận trước khi bắt đầu. Không tự đổi khoảng
checkpoint hoặc tăng góc vượt 30° khi thiếu quãng tiến.

# Cập nhật vận hành: ngưỡng ngang 5 cm, đoạn tiến chéo ngắn hơn

Giới hạn ngang khi xét đường thẳng và kiểm tra trục thân trong nhánh chấp nhận
sai số nhỏ đều là 50 mm; không còn giới hạn 20 mm ẩn trong nhánh này.
Toàn bộ hình bao hai càng và khoảng chừa mép vẫn phải lọt khe. Với khe hẹp,
ngưỡng 50 mm không bảo đảm mọi đường lệch dưới ngưỡng đều lọt.
Checkpoint đã chuyển ra 1,20 m từ gốc càng tới mặt trước, thay cho 1,10 m.
Cùng điểm xuất phát, đoạn tiến chéo giảm 10 cm theo phương pháp tuyến pallet.
Giới hạn góc và khoảng trống giữ nguyên; nếu đoạn ngắn không đủ sửa ngang,
không tự tăng góc hoặc bỏ kiểm tra va chạm. Quay 90° vẫn tắt.

# Cấu hình đang dùng: căn góc nhỏ/tiến chéo, không quay 90°

Người vận hành đã chọn trở lại phương pháp cũ. `quarter_turn_lateral: false`.
Ngưỡng xét nhánh trực tiếp là ngang dưới 50 mm, góc dưới 1,5°; quay bù nếu
cần rồi đo lại. Đây là ngưỡng xét nhánh, không bỏ giới hạn hình học thực tế:
toàn bộ hai càng, khoảng chừa mép và quỹ đạo phanh vẫn phải lọt khe. Giới hạn
lệch đường odom và các kiểm tra chống va chạm không đổi.
Phần phương pháp 90° bên dưới là lịch sử, không đang kích hoạt.

# Phương pháp sửa ngang đang chọn: quay 90°

`pallet_pickup.yaml` bật `quarter_turn_lateral: true`. Sau nhận diện, nếu nhánh
đi thẳng đã kiểm tra không phù hợp, kiểm tra toàn bộ chuỗi căn hướng pallet →
quay 90° về phía tâm lỗ → tiến sửa ngang → quay về hướng pallet. Quãng tiến
ngang tính theo mặt pallet trong odometry, giữ nguyên khoảng cách dọc mặt.
Giới hạn ngang 5–150 mm, góc ban đầu tối đa 12°, gốc càng cách mặt ít nhất
1,20 m; từng góc quét và quỹ đạo phanh giữ đầu càng ngoài mặt ít nhất 10 cm.
Kiểm tra laser toàn thân/càng trước khi chạy và tại mỗi chu kỳ; không bỏ
kiểm tra chiều cao, sensor, heartbeat, trôi tâm hay xác nhận nâng.

Xe dừng sau quay về, nhận diện mới tại chỗ rồi chốt đường đưa càng. Chuỗi này
thay đoạn tiến chéo, không tự chạy tới checkpoint 1,10 m. Có timeout 100 giây
cho sửa ngang vì hai lượt quay 90° chậm hơn đoạn căn góc nhỏ. Chưa chạy thử
trên xe thật. Hiệu chỉnh camera vẫn là thử nghiệm; cần quan sát lượt đầu.

# Chu trình pallet xanh hiện tại

Dùng phương pháp **đo lại ngoài pallet → chốt đường thẳng trong odometry → đưa
càng → chờ kiểm tra → nâng 8 cm**. Goal pickup bản đồ đã ngừng sử dụng.

Điểm đo lại đã chuyển từ 80 cm ra **1,10 m tính từ gốc càng tới mặt pallet**
vì hai chân ra khỏi mép dưới ảnh ở cự ly cũ. Xe đứng yên, mở phiên nhận diện
mới tại checkpoint, chờ YOLO tắt/RGB-D đạt rồi mới chốt đường đưa càng. Không
tiếp tục dùng bộ bám ảnh đã mất dấu trên đường tới checkpoint.

Nếu YOLO đã thấy đủ hai chân nhưng độ tin cậy chưa đạt 75%, lần suy luận kế
tiếp dùng khung rộng ít nhất 90% ảnh gốc quanh đề xuất để giữ ngữ cảnh. Mỗi
lần đều dùng ảnh mới; vẫn cần ba xác nhận đạt 75%, pallet xanh duy nhất và
phép đo RGB-D hợp lệ. Lỗi nhận diện in thêm độ tin cậy và lý do YOLO.

Sau mỗi phiên nhận diện (kể cả đo lại tại checkpoint), terminal in khoảng cách
gốc càng tới mặt pallet, tọa độ tâm lỗ theo xe (+y là trái), góc cần bù và
bề rộng khe đo được. Kết quả dùng phép đo mới đã bù RGB-D; lưu thêm vào
`recognition_results.jsonl` trong thư mục log chu trình.
Hồ sơ thử nghiệm hiện dùng bù ngang -13,010 mm từ mốc người vận hành đo
thanh đỏ trái xe 45 mm tại checkpoint (capture 14:36 ngày 04/11).
Đây là hệ số cố định áp dụng cho mọi phép đo, không phải mô hình theo cự ly.
Chỉ mới đối chiếu tại một tư thế; chưa xác nhận độ chính xác ở goal nhận diện
hoặc tư thế khác. Hồ sơ giữ `motion_qualified: false`; chạy thử phải có
`--allow-experimental-calibration`. Cờ này không bỏ các kiểm tra cảm biến,
quỹ đạo, vật cản, độ nhất quán pallet hoặc xác nhận trước nâng.

Terminal báo tâm lỗ theo gốc tại điểm giữa hai mũi càng, với chiều ngang
vẫn theo xe (+y bên trái xe). Với càng dài 39 cm và gốc càng x=36,55 cm,
mốc này ở x=75,55 cm, y=0 so với base_footprint: khoảng cách phía trước
giảm 75,55 cm, độ lệch ngang giữ nguyên. Đây không phải khoảng hở giữa một
càng với chân pallet hoặc khoảng cách tới trục pháp tuyến của pallet.

Chu trình giới hạn xử lý RGB-D 15 Hz và ảnh xem trước 2 Hz để giảm tải Pi.
Trước khi căn, xe chờ cửa sổ dữ liệu đạt và tiếp tục chờ nếu hết 30 giây.
Khi phiên nhận diện hết hạn hoặc mất dấu, chương trình thử nhận diện lại nếu
đã xác nhận xe đứng yên, càng trống và bộ điều khiển đã dừng. Các cửa sổ dữ
liệu vẫn phải đạt trước khi phát lệnh. Nếu Nav2 riêng của bộ căn không ACTIVE,
xe đứng yên và chu trình khởi động lại bộ căn tại chỗ rồi nhận diện lại.
Ctrl+C hủy chu trình.

Trong chu trình có giám sát, nếu RGB-D không tạo được cửa sổ ổn định trong
20 giây ở bước đo lại/căn góc cuối, xe giữ dừng rồi chuyển về nhận diện mới
ngay tại checkpoint. Mỗi yêu cầu có số thứ tự riêng để chu trình xử lý lại
đúng bước mà không lặp lệnh do trạng thái DDS cũ. Có thể nhận diện lại nhiều
lần; xe chỉ tiếp tục sau phiên mới và cửa sổ đo mới đạt. Trong lúc chờ,
odometry, laser, heartbeat, cao độ càng và việc xe giữ nguyên pose vẫn được
kiểm tra. Không khởi động lại đoạn tiến tới checkpoint hoặc đưa càng.

Nếu đo lại ở checkpoint báo góc pallet vượt giới hạn, chu trình kiểm tra xe
đứng yên, càng đã hạ và phép đo mới cho thấy gốc càng còn cách mặt pallet hơn
49 cm. Sau đó nó chuyển quyền điều khiển cho Nav2 về goal nhận diện đã lưu và
nhận diện/căn lại; có thể lặp lại. Nếu chưa xác nhận được điều kiện này thì
xe giữ dừng và chờ. Những lỗi khác, đặc biệt sau khi đã bắt đầu đưa càng hoặc
nâng hay khi không rõ trạng thái lệnh, vẫn dừng để tránh phát lại lệnh sai.

Tại checkpoint, phép kiểm tra xoay dùng giới hạn cho giai đoạn đầu càng còn
ngoài mặt pallet và kiểm tra toàn bộ đầu càng trong quỹ đạo phanh. Trước khi
xoay, chương trình dự kiến đường thẳng sau khi chỉnh góc. Nếu đường này lệch
trục hoặc càng sẽ chạm mép khe, xe về goal nhận diện để căn lại từ ngoài;
chỉ sau phép đo mới đạt mới được chốt đường đưa càng.

Sau lần đo cuối (kể cả sau quay), lỗi lệch trục/chạm mép của đường thẳng cũng
phát trạng thái `recovery_action=return_to_recognition`. Chu trình dùng trạng
thái này để về goal và làm lại khi xe còn ở `FINAL_ALIGN`; không phụ thuộc
vào câu thông báo lỗi. Lệnh dừng của người vận hành xóa trạng thái phục hồi.
Trước mỗi lần về goal vẫn xác nhận càng ở ngoài bằng dữ liệu mới; việc trở
về do Nav2 lập đường, không phát trực tiếp lệnh lùi/quay để ép càng vào.

Đường căn tới checkpoint cho phép hướng lệch tối đa 20° khi đầu càng còn ngoài
pallet, có kiểm tra toàn bộ thân/càng và quỹ đạo phanh với laser. Giới hạn căn
ban đầu vẫn 12°; đoạn đưa càng cuối giữ 0,5° góc và 5 mm lệch đường.

Mọi lượt đi tới checkpoint đều dùng `CheckpointApproach`, kể cả khi bộ căn
đã báo `ALIGNED`. Bản sửa 04/11 loại bỏ nhánh dùng bộ điều khiển đưa càng
cho đoạn tiếp cận này: nhánh đó có thể chặn quỹ đạo phanh vì giới hạn góc
trong khe dù đầu càng còn ở ngoài (log 10:14). Preview và chạy dùng cùng
quỹ đạo checkpoint, kiểm tra toàn bộ thân/càng với khoảng đệm laser 21 cm.
Theo xác nhận của người vận hành ngày 04/11, sau quay bù nếu còn lệch ngang,
`YAW_CORRECTED` được bàn giao sang kiểm tra đường tiếp cận checkpoint.
Chỉ khi dữ liệu mới, xe đứng yên, cao độ càng và toàn bộ quỹ đạo đạt mới báo
`CHECKPOINT_READY`; chu trình sau đó căn ngang ngoài pallet và nhận diện lại.
Không coi `YAW_CORRECTED` là đã căn đủ hoặc cho phép đưa càng.

Ở đoạn tiếp cận checkpoint, scan quá 400 ms làm xe dừng ngay nhưng cho phép
chờ phục hồi tối đa 3 giây. Chỉ tiếp tục sau scan mới, TF/odom/heartbeat hợp lệ,
xe đứng yên và dữ liệu đạt liên tục 0,8 giây, kiểm tra lại footprint/quỹ đạo.
Không đổi mốc odometry hoặc chốt lại đường để che sai lệch. Lỗi khác, vật cản,
mất dữ liệu kéo dài hoặc quá ba lần trễ thì hủy. Bấm dừng luôn hủy phục hồi.
Cơ chế này chỉ áp dụng khi tiếp cận checkpoint, đầu càng còn ngoài pallet;
đoạn đưa càng và nâng vẫn dừng/hủy theo các bảo vệ hiện có.

Khi quay tại checkpoint, mục tiêu giữ tâm xe ở vị trí bắt đầu lượt quay trong
odom. Bộ điều khiển dùng phản hồi vị trí/tốc độ dọc xe để bù trôi, lệnh bù giới
hạn ±12 mm/s; không tăng tốc độ quay. Độ trôi tâm xe và quãng trôi
khi phanh phải nằm trong 5 mm tính từ đầu mỗi lượt quay; tốc độ trôi tối đa
15 mm/s. Dao động tốc độ ngắn 5 mm/s không tự hủy khi vị trí vẫn đạt. Xe phải
đứng yên (|v| ≤3 mm/s, |w| ≤0,015 rad/s) đủ 0,8 giây trước bước tiếp theo.

Lệnh bù dùng tốc độ đã lọc 0,2 giây, giới hạn thay đổi 20 mm/s² để tránh đổi
dấu đột ngột. Khi dự báo trôi/phanh vượt 4 mm, xe ngừng quay và hãm, chờ đứng
yên rồi bù tâm về cách mốc gốc tối đa 1,5 mm trước khi quay tiếp. Mốc gốc không
được đặt lại. Trôi thực vượt 5 mm, không phục hồi trong 6 giây hoặc quá ba lần
hãm trong một lượt quay thì hủy. Quỹ đạo lệnh bù và vùng thân/càng vẫn kiểm tra
phanh, laser và khoảng cách pallet; không đưa càng khi đang phục hồi quay.

Sau khi đo lại, căn góc cuối chốt hướng pallet từ cửa sổ RGB-D ổn định rồi
bám hướng đó bằng odometry. Log 04/11 10:42 cho thấy cả lệnh 0,035 rad/s
cũng chưa làm xe quay rõ. Lệnh mới tăng dần từ 0,035 tới tối đa 0,060 rad/s
trong 2 giây để vượt vùng vận tốc thấp chưa đáp ứng; chưa xác nhận ngưỡng
thắng ma sát trên xe thật. Bộ điều khiển dự báo phanh theo tốc độ góc đo được,
giữ giới hạn trôi tâm 5 mm. Không còn hạn hoàn tất 6 giây: được tiếp tục khi
sai số cải thiện; nếu 20 giây không giảm thêm ít nhất 0,05° thì dừng báo
truyền động không đáp ứng. Các kiểm tra dữ liệu và va chạm vẫn áp dụng. Sau khi quay,
xe dừng và thu lại đầy đủ cửa sổ RGB-D 5 giây trước khi chốt đường đưa càng.
Kiểm thử bao gồm ngưỡng chết bánh 0,045 rad/s, căn chậm hơn 20 giây và
trường hợp truyền động không đáp ứng; chưa kiểm chứng trên xe thật.

Hướng dẫn khởi động, lệnh chạy và các giới hạn nằm trong [PICKUP.md](PICKUP.md).

Pallet có **bốn trụ ở bốn góc hình chữ nhật**, các trụ trước/sau thẳng hàng hai
bên. Phía sau hở, không có thanh chắn ngang. Khoang giữa trụ rộng 25,5 cm,
dài 35,5 cm theo hướng càng, cao 10 cm; khoảng trống ngoài phía sau là 30 cm.
Chạy lấy hàng với `--usable-depth-cm 35.5`.

Gốc càng mục tiêu cách mặt trước 2 cm; càng dài 39 cm nên phần càng đưa qua mặt
trước danh định là 37 cm. Toàn bộ hai càng vẫn phải lọt giữa hai hàng trụ trước
và sau, kể cả phần đầu nhô ra sau. Giữ khoảng chừa trước vật cản ngoài phía sau;
không dùng mặt phẳng qua hai trụ sau làm vách chắn giả.

Có thể thử riêng nhận diện/căn bằng `--from-current-view --alignment-only`, kết
thúc trước checkpoint/đưa càng/nâng.

Để tự định vị và chạy về goal nhận diện đã lưu trước khi lấy hàng:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_cycle.sh run --auto-localize --revalidate-recognition \
  --pickup-only --usable-depth-cm 35.5
```

Chỉ dùng goal nhận diện cho bước Nav2 ban đầu. Sau đó nhận diện/căn mới, đưa
càng trong odom; không dùng goal pickup bản đồ. Cờ `--revalidate-recognition`
cho phép đến goal nhận diện đang cần kiểm chứng bằng camera, không thay đổi
tọa độ/trạng thái đã lưu và không cho phép dùng goal giao chưa dạy.

Để tự định vị rồi lấy ngay từ vị trí hiện tại, thêm `--auto-localize` vào
`--from-current-view --pickup-only --usable-depth-cm 35.5`. Map/AMCL được mở cùng
`open_pallet_pickup_sensors.sh`; chương trình tự tìm pose khi đứng yên và yêu cầu
laser/pose/particle hội tụ ổn định 2 giây trước mọi bước nhận diện/căn/đưa càng.
`--auto-localize` kiểm tra vị trí hiện tại trước: nếu AMCL, TF và laser/bản đồ
vẫn khớp, giữ nguyên vị trí và in `GIỮ ĐỊNH VỊ`. Chỉ khi không đạt trong cửa sổ
kiểm tra 12 giây mới gọi định vị toàn cục. Kiểm tra dùng dữ liệu mới, không dùng
cờ “đã định vị” lưu trên đĩa; vì vậy sau reboot hoặc chuyển xe bằng tay vẫn
phải kiểm chứng lại. Mất laser/odom hoặc xe đang chạy thì dừng, không reset
AMCL khi đang di chuyển. AMCL tiếp tục bám vị trí bình thường giữa các chu trình.
Cửa sổ pose cho phép dao động 5 cm / 3°, tối thiểu 5 mẫu mới; không thay đổi
giới hạn dữ liệu quá hạn, khớp bản đồ, covariance hay phân bố particle.
Thất bại thì giữ dừng, không dùng pose lưu trước khi xe được chuyển bằng tay.

Chế độ lấy hàng là `--from-current-view --pickup-only`: làm việc từ vị trí
xe đang nhìn thấy pallet, không dùng pose bản đồ cho đoạn lấy hàng và không đi
giao. Sau khi đưa càng xong, gõ `nang` sau khi kiểm tra trực tiếp để nâng.

`--from-recognition` kiểm tra xe đã ở vùng goal và bỏ qua bước chạy về goal.
Hai chế độ `--from-*` không phải lệnh quay về goal nhận diện. Dropoff còn chờ
dạy, nên hiện chỉ dùng `--pickup-only` hoặc `--alignment-only`.

Các ghi chú của phương pháp cũ được lưu trong `docs/archive/`, không phải hướng
dẫn vận hành bản hiện tại. Kiểm thử mới dùng mô hình phần cứng giả trong ROS
domain 219; chưa xác nhận độ chính xác trên xe thật.

### Chấp nhận sai số nhỏ khi chốt đường thẳng

Sau đo mới tại checkpoint, nếu tâm lỗ theo trục ngang xe dưới 20 mm và góc
dưới 1,5°, thử kiểm tra toàn bộ đường thẳng với các ngưỡng này. Chỉ bỏ quay
bù nếu toàn bộ hình bao hai càng lọt khe và chiều sâu hợp lệ; giới hạn bám
đường odom vẫn giữ nguyên. Không bỏ checkpoint hoặc tự nâng không xác nhận.
Sau quay bù ở bước căn, thiếu cửa sổ RGB-D giữ xe dừng và thử lại kiểm tra
đường tại chỗ, không phát lại động tác quay và không chuyển ngay BLOCKED.

Nhánh DIRECT_READY được thử ngay khi bật căn lần đầu: ngang theo xe dưới
20 mm, cửa sổ RGB-D và tất cả interlock đạt, quỹ đạo quay tại chỗ và đường
đưa càng sau quay được kiểm tra đầy đủ. Nếu đạt, không tiến chéo hoặc tới
checkpoint; đo mới ngay tại pose nhận diện, quay bù nếu cần, đo lại rồi chốt
đường thẳng odom. Sau quay vẫn phải lọt khe; ngang ban đầu nhỏ không tự bảo
đảm ngang sau quay nhỏ. Nếu không đạt, dùng nhánh căn ngoài pallet hiện có.
Không tự nâng; giữ xác nhận người vận hành.
