# Theo dõi và đo pallet độc lập với YOLO

## Tìm pallet ở vị trí thay đổi, 2026-11-01

Detector tìm toàn bộ ảnh ở đầu mỗi lượt, dùng đề xuất YOLO để chọn vùng phóng
to trên **ảnh mới tiếp theo**. Vùng này theo vị trí/kích thước của đối tượng;
không lưu tọa độ pixel của pallet hiện tại. Mất mục tiêu trong vùng phóng to
thì trở lại tìm toàn ảnh. Mỗi lượt nhận diện lại bắt đầu từ toàn ảnh.

Đề xuất yếu chỉ phục vụ tìm kiếm. Seed vẫn cần confidence ≥0,75, mask không
bị cắt ở biên và đủ hai chân trước. Mask/bbox được đưa về ảnh RGB gốc 640×480
trước khi dùng với Depth và K/D; không thay intrinsics, TF hay hệ số bù để
làm góc nhỏ hơn. Có nhiều mục tiêu phù hợp thì báo chưa xác định được, không
tự chọn mục tiêu có confidence lớn nhất.

`--target-color green` chọn pallet xanh khi còn pallet gỗ khác trong ảnh;
màu chỉ lọc danh tính trong các kết quả YOLO, không tự tạo seed/mask hình học.
Vị trí pallet xanh có thể thay đổi. Bỏ tùy chọn này dùng chế độ `any`, yêu cầu
một pallet duy nhất phù hợp. Nếu có nhiều pallet giống nhau, cần chọn danh
tính mục tiêu trước khi căn; hệ thống không tự đoán pallet nào cần lấy.

Chân nhựa ít hoa văn được tìm thêm đặc trưng trong phạm vi tối đa 4 pixel
quanh từng chân. Vẫn giữ số đặc trưng tối thiểu, optical flow hai chiều,
inlier trên cả hai chân, tương quan ảnh, kiểm tra Depth và tuổi dữ liệu.

NCNN tự dùng phép tính FP16 khi CPU ARM có hỗ trợ, còn lại dùng FP32. Có thể
ép FP32 bằng `--ncnn-precision float32`. Model được warmup sau khi cấu hình
backend xong, trước khi tính thời gian nhận diện. Hai tiến trình dùng hồ sơ
FastDDS cho ảnh lớn giống camera; chỉ thay cách vận chuyển/xử lý, không nới
giới hạn đồng bộ hay độ mới.

Model chạy trên Pi được xuất từ cùng `models/best_v3.pt` với đầu vào mạng
288×384 để dành CPU cho Nav2 và đo RGB-D. Ảnh camera, mask seed và hệ tọa độ
đo vẫn là 480×640; loader lấy đúng kích thước mạng từ metadata của model.
Đường dẫn `data/runtime_models/best_v3_ncnn_model` trỏ tới bản chạy đã chọn;
bản cũ được giữ ở thư mục backup. Khi dựng máy mới, tái tạo bằng:

```bash
.venv/bin/python scripts/export_pallet_ncnn.py --activate
```

Kiểm tra nhận diện từ ảnh ghi thực, sau đó dịch ảnh trái/phải/lên trên và đổi
kích thước 0,8×/1,15×: cả sáu trường hợp có mask đủ hai chân, confidence
0,915–0,960, thời gian suy luận 158–265 ms trong lượt kiểm tra này. Đây là
biến đổi ảnh để kiểm tra tìm kiếm độc lập vị trí, chưa phải sáu tư thế xe thực.
Replay 50 cặp RGB-D gốc có hai mẫu xác nhận ban đầu, sau đó 48/48 hợp lệ.

Ghép RGB/Depth ưu tiên timestamp chụp, không ưu tiên callback vừa đến muộn.
Loại ảnh chụp quá 250 ms, không đưa timestamp RGB lùi vào tracker. Khởi động
nhận diện tự động chờ RGB mới đầu tiên rồi mới tính thời gian 3–5 giây.
Theo dõi quang học dùng vùng bao cả đặc trưng gốc và vị trí dự đoán, tự thay
đổi theo pallet; giữ hệ tọa độ pixel toàn ảnh và mọi ngưỡng kiểm tra cũ.
Seed trễ được nối qua các ảnh RGB đã quan sát với từng khoảng dưới giới hạn
500 ms; không dùng việc replay làm số đo Depth cũ trở thành mới.

Kiểm chứng sau cùng trên Pi cùng Nav2, xe/pallet đứng yên: YOLO tự tắt sau
xác nhận; 900/900 phép đo hợp lệ trong 60,00 s (15,00 Hz), khoảng ngắt lớn
nhất 165,8 ms. Tuổi phép đo 25,3–163,0 ms, RGB/Depth lệch −34,7…+17,6 ms.
Biên độ X 3,18 mm, Y 0,453 mm, hướng 0,959°. Tất cả tiêu chí của
`verify_pallet_handoff.py` đạt, gồm không có seed/suy luận YOLO mới và cửa
sổ cuối vẫn ổn định. Báo cáo local:
`data/diagnostics/pallet_adaptive_handoff_20261101_release.json`.
Các số đo/lượt chưa đạt ở những mục lịch sử bên dưới là phiên bản trước.

Giới hạn đã quan sát trước bước xác minh RGB bổ sung: chạy thêm bộ kiểm
thử ROS đồng thời trên Pi gây ngắt RGB 528 ms, làm hủy khóa mục tiêu.
Lượt 60 giây đạt ở trên chạy cùng Nav2, không kèm tải kiểm thử đó.

Khi RGB ngắt quá 500 ms, optical flow vẫn không được nối qua khoảng ngắt.
Mặc định có thể giữ **tham chiếu hình ảnh**, tối đa 1,5 giây, để xác minh
lại bằng hai phép khớp template độc lập trên ảnh hiện tại: NCC mỗi chân
≥0,90, hai chân phải có dịch chuyển nhất quán trong 2 pixel và đủ đặc trưng
RGB hiện tại. Không qua các kiểm tra này thì không có phép đo hợp lệ; hết
1,5 giây phải nhận diện YOLO lại. Khi xác minh thành công, xóa lịch sử hình
học, yêu cầu lại ba phép đo Depth mới và cửa sổ chất lượng 5 giây trước khi
cho phép căn xe. Không giữ lại khoảng cách/góc cũ thành dữ liệu mới, không
tự bật YOLO. Có thể tắt cơ chế này bằng `--no-rgb-gap-recovery` để dùng chính
sách mất tham chiếu ngay sau 500 ms như trước.

Camera đã chạy cùng Nav2: chỉ mở bộ nhận diện/đo, không mở thêm driver hoặc
bộ chạy căn xe thật. Từ thư mục `camera/detect_astra`:

```bash
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model --cpu-threads 2 \
  --target-color green --heading-floor-reference \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml
```

Lệnh này nhận diện ngay khi xe đứng yên. Để chờ goal nhận diện **mới** thành
công, thêm `--recognize-after-goal`. Không chạy song song hai bộ nhận diện.
Sau khi xác nhận, YOLO tự tắt và RGB-D được kiểm tra tiếp theo cửa sổ 5 giây
cũ. Điều kiện đứng yên đạt chưa thay cho kiểm chứng khi xe chuyển động; không
có lệnh motor hoặc goal điều hướng do các node nhận diện/đo phát ra.

Sửa góc giữa hai mặt phẳng bằng tham chiếu sàn, giữ nguyên vị trí:
[HEADING_REFERENCE.md](HEADING_REFERENCE.md). Web hiện độ lớn góc giữa mặt
trước xe và mặt trước pallet; các góc có dấu và hướng tới tâm lỗ giữ ở phần phụ.

Luồng chờ goal nhận diện mới, kiểm tra dữ liệu sau khi YOLO dừng và kết quả
camera ngày 2026-11-01: [PERCEPTION_HANDOFF.md](PERCEPTION_HANDOFF.md).
Phạm vi hiện tại chưa chạy căn xe thật; lượt kiểm chứng 60 giây mới nhất còn
hai tiêu chí chưa đạt dù 1530/1530 phép đo đều hợp lệ.

## Hai tiến trình

- `scripts/publish_pallet_detection.py`: chỉ nhận RGB và chạy YOLO. Phát `PalletDetection` trên `/pallet/detection`, gồm RGB gốc, mask PNG cùng kích thước/timestamp, bbox và nhãn pallet. Có dịch vụ `/pallet/set_enabled` và heartbeat `/pallet/enabled`. Không phát `/pallet/target`.
- `scripts/publish_pallet_target.py`: không import Torch/Ultralytics, không nạp model. Nhận RGB + Depth đăng ký, CameraInfo, kiểm tra đăng ký và mask khởi tạo. Phát `/pallet/target` và ảnh theo dõi `/pallet/preview/compressed`.
- `scripts/run_pallet_perception.py`: supervisor mở hai tiến trình, dừng cả hai khi Ctrl+C. `open_pallet_target.sh` gọi supervisor; các tham số model, bù, camera cũ vẫn dùng được.

YOLO cung cấp danh tính/vùng pallet; không giữ ảnh Depth hoặc phép đo từ lần nhận diện làm số đo hiện tại. Trên mỗi cặp RGB-D mới, tracker tìm lại vị trí hai chân trong RGB, sau đó gọi cùng `measure_pallet_target` và hồ sơ bù đã dùng trước đó với Depth **của cặp hiện tại**. Cả hai chân phải có đủ mẫu hợp lệ trên cùng dải hàng; ngưỡng mẫu, tỷ lệ hợp lệ, MAD và ngưỡng tuổi 500 ms giữ nguyên.

## Kiểm tra theo dõi

Optical flow được kiểm tra hai chiều, lọc ngoại lai và yêu cầu hỗ trợ trên cả hai chân. Mỗi khung hình đăng ký lại với RGB gốc lúc khởi tạo, dùng vị trí trước làm dự đoán; không cộng dồn mask từ khung này sang khung khác. Kiểm tra tương quan hình ảnh riêng mỗi chân tránh duy trì vùng cũ khi pallet biến mất. Chuyển động lớn, sai tỷ lệ, mất một chân, ra biên ảnh hoặc khoảng ngắt RGB trên 500 ms làm mất theo dõi. Lỗi mẫu Depth chỉ làm phép đo hiện tại không hợp lệ; khi RGB còn theo dõi được, giữ danh tính pallet và xác nhận lại ba phép đo mới khi Depth trở lại. Một lỗi CameraInfo/đăng ký tạm thời cũng không xóa trạng thái RGB; trong thời gian đó không xuất vị trí/góc.

Khi một phép đo lỗi, topic phát `valid=false` ngay và web xóa số đo. Trong khoảng ngắt ngắn, trạng thái hình ảnh nội bộ có thể còn để nối lại nhưng không được xuất thành số đo hợp lệ. Khi thực sự mất theo dõi RGB, bật YOLO để khởi tạo lại. Lỗi đồng bộ/Depth tạm thời tự phục hồi nếu RGB vẫn theo dõi được; số đo phải qua xác nhận lại trước khi hợp lệ. Không có tái nhận diện ngầm khi người dùng tắt YOLO.

Số đo hợp lệ có `status` bắt đầu `TRACKED_RGBD`; `seed_age_s` là tuổi nhận diện ban đầu, không phải tuổi Depth/số đo. `header.stamp` vẫn là thời gian chụp RGB của cặp hiện tại. `yolo_confidence` là NaN vì khung theo dõi không có suy luận YOLO mới. Cần kiểm tra `valid` và thời gian như trước.

## Build và chạy

Interface mới cần build một lần trước khi chạy phiên bản này:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select detected_pallets_interfaces --executor sequential
```

Sau đó dùng ba terminal như README: camera, bộ nhận diện/đo, web. Ví dụ terminal nhận diện/đo:

```bash
cd /home/admin/tai_robot_one/tai_robot_one_community/camera/detect_astra
bash scripts/open_pallet_target.sh \
  --model data/runtime_models/best_v3_ncnn_model \
  --cpu-threads 2 --max-fps 2 --track-max-fps 0 --preview-fps 8 \
  --conf 0.60 --nms-iou 0.30 \
  --measurement-bias-yaml config/calibration/pallet_target_bias_2026_10_31.yaml
```

`--max-fps` giới hạn riêng YOLO. `--track-max-fps 0` đo nhanh nhất theo ảnh nhận được; JPEG web mặc định tối đa 8 FPS. Web luôn dùng ảnh/số đo cùng timestamp và hiển thị tổng CPU cả hai tiến trình. Bật/tắt YOLO không làm xóa bộ lọc góc hoặc phép đo RGB-D còn hợp lệ.

Sau khi model sẵn sàng, YOLO tự nhận diện 3–5 giây rồi tắt. Đủ ba kết quả rõ cùng phép đo RGB-D ổn định mới xác nhận pallet; hết 5 giây mà chưa đủ thì báo nhận diện lại. Khi đã xác nhận, xe chờ bấm căn chỉnh và bộ đo tiếp tục độc lập. Nút **Nhận diện lại (3–5 giây)** bắt đầu lượt mới, yêu cầu seed và phép đo mới; mất dấu không tự bật YOLO. Theo dõi vẫn chạy; khi YOLO bật, ảnh có nhãn Box/Pallet/Empty pallet được ưu tiên ở khung chính, có chú thích màu và trạng thái bên ngoài ảnh. Ảnh đo có thể mở trong **Xem thêm ảnh đo RGB + Depth**. Khi tắt YOLO, khung chính chuyển sang ảnh theo dõi RGB-D; ảnh không ghi đè khoảng cách/góc lên góc trái.

## Phạm vi hiện tại

Đã có theo dõi và đo, lọc góc trên web, báo không cần chỉnh hướng khi |góc sau lọc| < 2°.
Bộ căn riêng `scripts/align_pallet.py` tạo điểm chờ trong `odom`, mặc định chỉ xuất vận tốc đề xuất.
Bộ chạy mới chọn goal gần nhất trên pháp tuyến pallet, giữ mặt pallet → gốc càng tối thiểu 1,20 m (chừa 10 cm), rồi sửa góc; lệch ngang dưới 2 cm, góc dưới 2°. Xem ALIGNMENT.md cho luồng Nav2 mới.
Xem [ALIGNMENT.md](ALIGNMENT.md) về nút căn riêng, lớp tránh vật cản và điều kiện chạy thật.
Các node nhận diện/đo không phát lệnh xe. Chưa có tiến càng trong vùng mù hoặc nâng hàng.

Theo dõi RGB phụ thuộc độ rõ ảnh, ánh sáng, che khuất và thay đổi góc/kích thước. Kiểm thử ảnh tổng hợp và thử xe/pallet đứng yên không xác nhận độ tin cậy khi xe chạy; cần đo tiếp với chuyển động thực trước khi nối bộ điều khiển.

## Đo thực tế trên Pi 5, 2026-10-31

Sau khi khởi tạo bằng YOLO, đã gọi dịch vụ tắt YOLO và giữ xe/pallet đứng yên: **523/523 phép đo hợp lệ trong 20,04 s (~26,09 FPS)**, không có phép đo lỗi trong cửa sổ đó. CPU toàn máy trung bình **35,77%** trên thang bốn lõi = 100%; riêng tiến trình đo khoảng 15,78%, tiến trình YOLO khoảng 2,99% (bao gồm thời điểm chuyển chế độ và xử lý thông điệp camera). Web nhận ảnh khoảng 7,45 FPS, tuổi số đo kiểm tra cuối 58 ms. Đây là phép thử đứng yên; không suy ra độ chính xác/độ tin cậy khi xe di chuyển hay khi pallet bị che. Chi tiết local: `data/diagnostics/independent_rgbd_live_verify.json`.

Lượt chạy tiếp 45,04 s sau khi bổ sung chẩn đoán: **1149/1150 phép đo hợp lệ (~25,51 FPS)**; một phép đo không hợp lệ vì đang chờ cặp RGB/Depth đồng bộ, sau đó tiếp tục theo dõi mà không bật YOLO. CPU toàn máy trung bình 35,88%, web khoảng 7,64 FPS và tuổi số đo cuối 28 ms. Chi tiết: `data/diagnostics/independent_rgbd_long_verify.json`. Một lượt thử dài trước đó mất dấu trong thời gian chạy thêm kiểm thử; nguyên nhân của lượt đó chưa được ghi lại, nên các số trên không phải cam kết theo dõi liên tục dưới mọi tải CPU.

## Sửa lỗi CameraInfo làm mất mục tiêu sau khi tắt YOLO

Trước đây, `process_pair` gọi `estimator.reset()` ngay khi CameraInfo không khớp RGB trong 50 ms. Vì YOLO đã tạm ngừng, không còn mask mới để khởi tạo lại dù RGB vẫn nhìn thấy pallet. Bản sửa ưu tiên ghép đủ bộ RGB/Depth/CameraInfo, chờ tối đa 50 ms theo thời gian nhận callback trước khi bỏ qua phép đo của bộ thiếu metadata. Hàng đợi subscriber CameraInfo tăng từ 1 lên 16; ngưỡng lệch timestamp CameraInfo/RGB vẫn là 50 ms. Khi thiếu metadata, chỉ tiến trạng thái theo dõi trên RGB mới, xóa lịch sử xác nhận phép đo và phát `valid=false`. Khi có đủ metadata hợp lệ, cần ba phép đo mới ổn định trước khi phát `valid=true`. Không đổi timestamp, không sửa K/D, không dùng Depth thô hay kéo dài tuổi phép đo. Kiểm thử hồi quy tái hiện CameraInfo lệch 70 ms liên tiếp và xác nhận đo phục hồi mà không có seed YOLO mới.

Xác minh bản sửa: 32 kiểm thử đạt, gồm callback CameraInfo đến sau ảnh, ưu tiên bộ dữ liệu đầy đủ, giữ nguyên ngưỡng 50 ms và khôi phục đo sau CameraInfo lệch 70 ms mà không cấp seed YOLO mới. Thử camera thật sau khi tắt YOLO: **2381/2381 phép đo hợp lệ trong 90,02 s (~26,45 FPS)**, không có seed YOLO mới trong cửa sổ này. Web cuối có số đo mới 57 ms, ảnh khoảng 7,39 FPS. Đây là thử đứng yên; lỗi CameraInfo được tái hiện ở kiểm thử hồi quy, không xảy ra tự nhiên trong cửa sổ 90 s này. Chi tiết local: `data/diagnostics/camera_info_recovery_live.json`.

## Kiểm tra góc dao động và web gián đoạn sau khi mở lại, 2026-10-31

Xe/pallet được người vận hành xác nhận giữ nguyên. Lúc kiểm tra ban đầu, góc node đã khoảng +2,7° và góc web sau lọc khoảng +3°; hồ sơ bù vẫn đúng bản cũ. Không đổi hệ số bù, vị trí camera, hệ trục hoặc ép góc về 0 trong lượt sửa này.

Phát hiện `RGBDTargetEstimator.process` vẫn nhận thay mask tham chiếu trên mỗi seed YOLO mới khi RGB đang theo dõi được. Mask khác nhau làm thay vùng lấy Depth dù cảnh đứng yên. Sửa để seed chỉ khởi tạo khi chưa có/mất theo dõi RGB; khi đang theo dõi, mask được đưa sang RGB hiện tại từ cùng một tham chiếu. Khi YOLO bật, nhãn/ảnh detector vẫn cập nhật riêng. Kiểm thử mới thay mask YOLO lên/xuống liên tiếp và xác nhận tham chiếu giữ nguyên, Depth mới vẫn thay đổi khoảng cách, và sau reset vẫn nhận seed mới.

Bổ sung OpenMP chờ thụ động, NCNN `openmp_blocktime=0`, ưu tiên CPU thấp hơn cho riêng tiến trình detector (`nice +5`). Đã thử hạ YOLO xuống 1 FPS để đối chứng; theo yêu cầu người vận hành, mặc định và các lệnh chạy đã được đặt lại **2 FPS** (`--max-fps 2`). Bộ đo RGB-D vẫn không giới hạn. Các kết quả 1 FPS bên dưới chỉ là lịch sử thử nghiệm.

Đo thực tế các cửa sổ liên tiếp (khác thời điểm, không phải benchmark kiểm soát mọi tải nền):

| Chế độ | CPU toàn Pi trung bình | Phép đo hợp lệ / tổng | Góc sau lọc trên web |
| --- | --- | --- | --- |
| Trước sửa, YOLO 2 FPS, 15 s | 89,6% | 91 / 104 | +0,47 … +1,16° |
| Sau sửa mask/lịch CPU, YOLO 2 FPS, 15 s | 77,9% | 134 / 144 | +0,15 … +1,16° |
| Sau sửa, YOLO 1 FPS, 30 s sau có mục tiêu | 60,4% | 417 / 463 | +0,60 … +1,11° |
| Sau sửa, YOLO tạm ngừng, 45 s | 39,5% | 1051 / 1052 | −1,77 … −0,98° |

Khi YOLO tạm ngừng: không có seed mới, tốc độ phép đo hợp lệ khoảng 23,4 FPS, **175/175 lần đọc web có số đo**. Một thông điệp chờ đồng bộ vẫn được phát invalid, không giữ thành số đo mới. Góc thô trong cửa sổ đó là −2,44 … +0,31°; góc đã lọc vẫn dưới 2° về trị tuyệt đối. Sự thay đổi theo thời gian khi cảnh đứng yên cho thấy còn nhiễu/sai lệch phép đo; không được dùng việc đứng yên để tự gán góc chuẩn bằng 0.

**Chưa loại bỏ hết gián đoạn khi YOLO bật**: cửa sổ 1 FPS vẫn có hai lần khoảng ngắt RGB trên 500 ms và thời gian nhận diện lại. Không nới tuổi phép đo/đồng bộ để che các lỗi này. Khi căn xe, dùng chế độ đã chọn: nhận diện/khóa pallet, tự tạm ngừng YOLO, tiếp tục đo RGB-D. Cần kiểm tra thêm trước khi dùng YOLO và bộ đo liên tục dưới tải điều hướng.

Chi tiết local: `data/diagnostics/stability_before.json`, `stability_fixed_on.json`, `stability_fixed_1fps_steady.json`, `stability_fixed_yolo_paused.json`. Lượt thực tế này không chạy động cơ hoặc thay vị trí xe.

## Luồng nhận diện 3–5 giây rồi chờ căn

Theo yêu cầu mới, nhận diện khởi động mặc định ở 2 FPS sau khi model sẵn sàng. Bộ `pallet_acquisition.py` kiểm tra ba kết quả YOLO rõ/nhất quán cùng chuỗi phép đo RGB-D hiện tại trước khi tự tắt YOLO từ giây 3; đến giây 5 chưa đủ thì dừng và báo nhận diện lại. `/pallet/recognition/status` lưu trạng thái lượt nhận diện. Lượt mới yêu cầu seed và phép đo chụp mới, không tái sử dụng xác nhận của lượt trước. Sau xác nhận, RGB-D tiếp tục đo và xe chờ nút căn. Nút căn gọi `/pallet/prepare_alignment`, chờ suy luận dở kết thúc và ít nhất ba phép đo chụp sau xác nhận tắt YOLO. Không tự thực hiện các goal nhận diện/đưa càng/nâng hàng. Đã xác minh bằng 59 kiểm thử; xem ALIGNMENT.md. Các số đo thực tế trong những mục trước là lịch sử của các phiên bản trước luồng tự tắt này.
