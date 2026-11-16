# Giao diện mô phỏng và nhận diện

Từ gốc repo, mở giao diện desktop:

```bash
./simulation/launch_r3_sim.sh gui
```

Chạy xong muốn đổi vị trí: sửa ba ô → **Áp dụng / chạy lại vị trí** → **Tạo quỹ đạo** → **Chạy bám mô phỏng**. Không cần reset terminal. Đổi ca sẽ xóa đường/kết quả của ca cũ; đồ thị trước khi tạo đường hiển thị chú thích chưa có quỹ đạo. Nút Đo lại camera chỉ thử nhận diện lại tại vị trí hiện tại, không dịch chuyển xe và không thay thế bước Tạo quỹ đạo.

Đồ thị giao diện và hình mặc định cho ca tự nhập dùng **world cố định**: pallet giữ nguyên vị trí/hướng, xe thay đổi theo ba ô nhập. Hình xám là xe/càng lúc đầu; vẫn chỉ có hai đường planned/thực tế. Đích RGB-D đo được không xoay hình pallet vật lý. CSV điều khiển và log vẫn theo odom. [Ảnh kiểm tra hệ world](../output/r3_world_view_fix/three_start_poses.png).

Pallet r3 nay dùng màu vật liệu **#25A18E** theo yêu cầu. World đang mở cần bấm Áp dụng lại để nạp màu. Thử A/B cùng vị trí1,5m/model/ngưỡng cho0 phát hiện ở cả màu cũ và màu mới; đổi màu riêng lẻ chưa làm YOLO nhận được. [Ảnh và báo cáo thử màu](../output/r3_color_25a18e/README.md).

Nhập **khoảng cách tâm gốc càng đến mặt pallet** (m, dọc pháp tuyến), **lệch ngang so tâm lỗ vào** (m, dương sang trái pallet), **góc lệch** (độ, dương quay trái). 0° nghĩa càng hướng vào pallet và mặt phẳng gốc càng song song mặt pallet. Số nhập hữu hạn không có min/max; nhận cả số âm và góc ngoài ±180°, góc được quy về cùng hướng vật lý. Bản đồ planner và sàn mô phỏng mở rộng theo tư thế. Chấp nhận số nhập không bảo đảm tư thế không va chạm hoặc solver tìm được đường.

1. Bấm **Áp dụng tư thế / mở world**. Có thể bật cửa sổ Gazebo 3D; mặc định xem camera và đường ngay trong ứng dụng.
2. Xem **Camera trực tiếp**, **Ảnh nhận diện cuối** và kết quả YOLO/RGB-D: loại pallet, độ tin cậy, khoảng cách, lệch ngang, góc, trạng thái và mốc chụp. Chưa đo được thì hiện dấu “—” cùng lý do.
3. Chọn **Pose nhập** để thử planner/follower theo kịch bản, hoặc **RGB-D đã khóa** để dùng phép đo camera đã ổn định. Không thay số đo camera bằng tư thế nhập. YOLO hiện chưa nhận ra pallet hình hộp của world kiểm tra; nguồn RGB-D chưa có pose khóa. Không đổi model hay thêm detector thay thế.
4. Bấm **Tạo quỹ đạo**, xem đường xanh, rồi **Chạy bám mô phỏng**. Giao diện chỉ vẽ hai đường của **tâm gốc càng**: bộ tạo quỹ đạo và thực tế Gazebo (truth độc lập). Odom vẫn ghi CSV phục vụ đối chiếu, không thêm đường lên giao diện. Khi tạo xong đường hoặc bắt đầu chạy, tắt đo pallet; camera tiếp tục truyền ảnh và giữ kết quả đo đứng yên cuối. Recorder truth không đưa dữ liệu vào bộ bám.
5. Bấm **DỪNG MÔ PHỎNG** để hủy. Đổi tư thế bằng Áp dụng lại; giao diện chỉ đóng world do chính nó mở. Nếu có world r3 khác, đóng world đó trước. Đóng giao diện sẽ dừng các tiến trình nó tạo.

Không có thời hạn chạy xe tổng cố định. Giữ các điều kiện dừng vật cản, dữ liệu stale, va chạm và giới hạn v≤0,15 m/s, ngang≤20 mm, góc≤0,5°. Giữ goal danh định gốc càng 20 mm; sai số dọc chỉ báo cáo theo yêu cầu. Nút Chạy không nâng càng. `mission` qua CLI vẫn thử nâng80mm; vấn đề thiếu hành trình nâng đã báo trước chưa được xử lý bằng đổi controller.

Chạy hai terminal vẫn được; terminal 1 sẽ hỏi ba giá trị nếu chưa truyền:

```bash
./simulation/launch_r3_sim.sh world
# Hoặc: ./simulation/launch_r3_sim.sh world --distance 2 --lateral .25 --yaw-deg 10
# Terminal 2:
./simulation/launch_r3_sim.sh run
```

Mỗi lượt xuất vào `simulation/output/r3_user/`. Hình mặc định `trajectory.png/svg` có hai đường. Xem [ảnh camera và giao diện](../output/r3_gui_validation/gui_live_camera.png), [ảnh lượt lệch góc đã chấm](../output/r3_gui_validation/gui_camera_paths.png) và [kết quả kiểm chứng](../output/r3_gui_validation/README.md). Ca thẳng đạt; ca 2m / +0,25m / +10° hoàn tất nhưng FAIL ngang truth khoảng30,4mm, không tuyên bố mọi tư thế đều đạt.

# r3-approved-lattice implementation

Read `../../pi5_docking_diagnostics/TRIEN_KHAI_R3.md` for the actual results and limitations.
The approved implementation uses the **installed Nav2 SmacPlannerLattice** with differential primitives,
**installed Nav2 ConstrainedSmoother** on translating approach segments only, then an exact fork-axis line.
The line is never optimized. No alternative planner is used on failure.

The existing `ShadowFollower` Pure Pursuit equation steers translation. Stationary yaw correction calls the
existing `pallet_alignment_core.stationary_heading_command`; only its existing equation was extracted.
Speed profiling respects0.15m/s, omega0.20rad/s, acceleration0.08m/s² and angular acceleration0.40rad/s²
from the existing demo/alignment config. There is no minimum-speed floor. The fork transform, target,
geometry and primitive/config hashes are frozen with each plan. A new output directory is required to
avoid overwriting reviewed plans. A curvature discontinuity at A is handled by a stopped phase boundary,
not by changing the exact terminal line.

## Run exact-input kinematic simulation

Requires ROS2 Jazzy, nav2_smac_planner, nav2_constrained_smoother, nav2_costmap_2d, pluginlib,
CMake/C++17, nlohmann_json, Python numpy/matplotlib. The C++ helper is built under `/tmp` and loads the
real plugins; it has no drive/lift command publishers.

```bash
source /opt/ros/jazzy/setup.bash
python3 simulation/r3_sim.py
python3 -m unittest discover -s simulation -p 'test_*.py'
python3 simulation/r3_native/run_tests.py
```

Outputs go to a fresh `simulation/output/r3_runs/<local timestamp>/` directory. `fixture.json` explicitly
marks estimated fork/pillar positions and synthetic base/housing offsets. Do not use it as real calibration.
The5 exact-pose cases do not establish RGB-D accuracy, wheel slip, contact, or hardware acceptance.
A separate historical quintic comparison is read-only:

```bash
python3 simulation/r3/compare_baseline.py <kinematic-run-directory>
```

## Run isolated Gazebo

Build the current package as documented in `../README.md`, using a separate `/tmp/tai_r3_ros_install`.
Generate the simulation-only URDF/world with approved dimensions:

```bash
python3 simulation/r3/generate_gazebo_assets.py
source /opt/ros/jazzy/setup.bash
source /tmp/tai_r3_ros_install/setup.bash
export ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=tai_r3_sim_only_89
export TAI_SIM_ROBOT_XACRO=/tmp/tai_r3_gazebo_assets/description/robot.urdf.xacro
export TAI_SIM_BRIDGE_PALLET_POSE=1 TAI_SIM_DISABLE_POINTS=1
ros2 launch tai_robot_one gazebo_sim.launch.py headless:=true \
  use_rviz:=false use_phone_teleop:=false \
  world:=/tmp/tai_r3_gazebo_assets/world.sdf spawn_x:=0.0225 spawn_y:=0.10
```

Use this spawn only with the `straight` exact-input case. Start the independent scorer **before** the
trial; omit --duration for an unlimited recorder (the launcher handles its lifecycle). In terminals with the same isolated environment:

```bash
python3 simulation/r3_gazebo_scorer.py --output /tmp/my_r3_trial
python3 simulation/r3_gazebo_trial.py --gazebo-execute \
  --plan <kinematic-run>/straight/frozen_plan.json --output /tmp/my_r3_trial
python3 simulation/r3_score_gazebo.py /tmp/my_r3_trial \
  --plan <kinematic-run>/straight/frozen_plan.json
```

The controller checks domain/partition, LOCALHOST discovery, simulated controller clock, single
clock/odom/joint publishers and a matching start pose before creating command publishers. The scorer
reads named Gazebo model poses in a separate process and never feeds control. Full contact-force and
RGB-D-obstacle instrumentation are not implemented in this test adapter. All Gazebo results remain
simulation evidence, not real-operation approval.

Stop the launch after the trial and verify that its Gazebo server has exited before starting another
world in the same partition. A server may survive its launcher; mixed clocks make a run invalid.

## Real stationary-camera preview

`r3_camera_preview.py` has no drive/lift publishers or execute flag. It accepts an exported stationary
camera session, confirmed calibration and geometry, requires post-YOLO-off distinct RGB-D samples,
freezes the target, and exports a planned-only preview. It rejects the synthetic fixture, missing
extrinsics, or an independently measured yaw bound above0.5°. It cannot certify absolute accuracy
from sample stability alone. `corrected_base_scalars` and `optical_SE3` are separate adapters, so an
optical transform is never applied twice to existing corrected scalars.

```bash
source /opt/ros/jazzy/setup.bash
python3 simulation/r3_camera_preview.py --measurement-json <real-session.json> \
  --calibration <confirmed-calibration.json> --geometry <confirmed-geometry.json> \
  --output <fresh-preview-directory>
```

The live ROS acquisition lifecycle now has a separate supervised trial path in
`../r3_live_prepare.py` and `../run_r3_live_trial.sh`. It captures recent stationary RGB-D,
including a fresh valid frame after YOLO stops,
uses the camera's one +2.5 cm lateral correction, and creates a new provisional plan from the
current control pose (map-corrected odom in the default AMCL mode). The pallet heading uses a fresh floor-referenced front-plane estimate instead of the old
fixed depth-skew coefficient. The A→C path finishes its bounded quintic bend 12 cm before C and stays straight to C; C→B is also straight. Live A→C plans use signed curvature feedforward plus heading and lateral error feedback at the nearest path reference. The same law continues through the final 12 cm, retaining lateral correction instead of switching abruptly from Pure Pursuit to heading-only control. The one-command real script passes `--stop-at-c`: it brakes before C, waits up to 2 s for the wheels to settle, and ends without a second camera measurement, straight insertion, or lift. In odom-only mode it uses the EKF `/odom` pose for X/Y and control yaw; the default AMCL mode uses bounded map-corrected odom for path control. Raw IMU yaw is logged for comparison. Missing IMU or more than 2° disagreement between raw IMU and fused odom yaw stops the trial. The geometry is explicitly
approximate (`confirmed_real_geometry=false`) and requires `--allow-provisional`; the trial speed
is limited to 0.10 m/s (operator requested), with curvature, acceleration and braking speed limits. The guarded follower and pallet-masked LiDAR scan are in
`../r3_real_trial.py` and `../r3_real_obstacle_filter.py`. It does not command the lift.
The one-terminal script starts the missing camera/drive/LiDAR/perception processes and stops only
those it owns. After the run, `../r3_render_real_result.py` saves `trajectory_result.png` and SVG
showing the initial robot, pallet, planned base path, planned fork-root path, and recorded base
path. In AMCL mode it also shows raw odom for comparison; the final outline uses the actual control pose. The image title records `DONE_TRACKING` or the actual stop state. The initial target must be
fresh when planning and starting the safety filter; the Enter review wait has no time limit and
does not remeasure the pallet. Motion still requires current odom, IMU and LiDAR at A. The stop report saves final IMU and odom heading errors, their disagreement, estimated lateral position and fork-tip clearance. The plotted orange final robot is a sensor-based estimate, not a physical measurement of fork-to-pallet alignment.
RGB-D floor correction is now shared by position and heading; earlier trials mixed a static position correction with a per-frame heading correction. The floor method still assumes zero camera roll and unverified camera mounting yaw. It does not establish absolute alignment from temporal stability. Acquisition stores bounded raw capture evidence under `camera_evidence/`, scalar inputs under `plan/capture_samples.json`, and raw wheel/fused odom/IMU under `trial/odom_imu_diagnostics.jsonl`. Capture timestamp differences and calibration assumptions are explicit.

For a stationary reference check at A without starting trajectory tracking:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --capture-only
```

This starts acquisition/hardware odom sources, saves evidence after planning, then cleans up its processes. Use a physically checked pallet pose that remains visible to the camera. The old empirical forward residual is not independently validated for the new depth correction.

The current pictured entered-pose plan remains blocked. See `../../camera/detect_astra/ALIGNMENT.md`.

## Read-only preflight for the pictured A→C→B plan

From the repository root, inspect the frozen plan used for the base/fork-root image:

```bash
python3 simulation/r3_real_preflight.py --plan \
  simulation/output/r3_camera_measured_preview/centered_20261111_145707/plan_refined/frozen_plan.json
```

The command makes zero robot commands and exits with code 2 while any static real-plan blocker remains.
The pictured plan currently uses an entered pose and unconfirmed synthetic geometry. The historical
alignment launch has been retired; its command does not execute this A→C→B plan.

## Firmware

Both standalone `cang_nang_ha` and production `bno055_imu_monitor` implement bounded1mm HOME_SETTLE,
then zero. Lower-active startup requires release/reapproach. Normal downtravel remains blocked by the
lower switch and no longer changes the settled zero at the earlier trigger. Watchdog, faults, upper stop
and existing debounce values are retained. Native tests compile the actual lift source with deterministic
IO stubs; they verify exact step count, final zero/lift80mm and fault paths, not physical step loss/timing.
Both projects have been built for ESP32; neither has been flashed.

### Empirical fork-axis lateral residual (2026-11-12)

The separate residual profile `real_trial_lateral_calibration.json` is currently **disabled (0.000 m)**. A +0.025 m axis shift was tried from approximate operator observations, then disabled after trials `153159_MwqW6S` and `153415_nTMQCp` produced a reversed leftward miss and worse heading. The original physical camera mounting correction remains +0.025 m.

A perpendicular offset depends on measured pallet heading as well as translation: at 1.4 m, one degree of heading error changes the offset by about 2.4 cm. Do not identify a constant translational calibration from one approximate perpendicular measurement. The separate report and once-only correction validation remain available, but the live default applies no additional residual. New invocations recognize and freeze a fresh target. Historical plans are unchanged.


## LiDAR/map localization and a newly taught recognition A

The one-command trial now starts at the robot's current position and uses AMCL with the edited real map to bound odom drift during camera measurement and A→C tracking:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh
```

Without an explicit initial pose or RViz, this command now searches the selected map directly against current LiDAR scans before seeding AMCL. Two separately captured stationary scans must agree; sampled alternative poses and held-out beams must pass consistency checks. The global bootstrap has a 150-second bound, followed by the configured AMCL convergence wait (120 seconds by default). Rejected or ambiguous searches stop before any trajectory starts. An approximate measured pose can still be provided explicitly:

```bash
read -r -p 'Map X Y yaw(deg): ' X Y YAW_DEG
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --initial-pose "$X" "$Y" "$YAW_DEG"
```

To allow a longer search, run `bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --localization-timeout 180`. To run the previous odom-only diagnostic explicitly, run `bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --localization odom`.

The default map is `ros2_ws/maps/tai_map_real_edited.yaml`. AMCL owns `map -> odom`; the EKF owns `odom -> base_footprint`. No previous recognition point or saved AMCL pose is imported. A plain trial starts where the robot stands; it does not automatically drive to saved A.

### Teach A later

Place the robot at the NEW desired camera recognition position and heading. This command starts localization and saves that stationary map pose; it does not start Nav2 or command wheel motion:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --save-a
```

It creates the local, Git-ignored `simulation/r3/recognition_A.json` only after fresh stationary localization passes. The schema, UUID/save time, map geometry and image hash distinguish it from legacy waypoints. Existing files are not overwritten automatically; use `--a-file /absolute/path/new_A.json` to teach a separate new point.

### Automatically localize, navigate to new A, then measure pallet

After A has been taught:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --nav-to-a
```

When the operator has already placed the robot at the recognition area and
wants to measure the pallet without a Nav2 positioning adjustment, use:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh && \
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --recognize-here --camera-obstacles
```

This mode starts the precision AMCL profile and continuous localization monitor,
validates the current stationary pose twice, and hands the same localization
process over to the trial frame. It never starts Nav2 or sends a navigation
goal. `recognition_here.json` records the actual estimated pose and its error
relative to saved A, including `within_saved_A_limits=false` when appropriate;
it does not assert that A was reached or replace A's coordinates. Pallet
measurement and the later C trajectory use the actual pose. The normal motion
confirmation remains before the C trial. Add `--capture-only` to measure and
save evidence without starting that motion controller. `--recognize-here`
cannot be combined with `--nav-to-a`, `--nav-check-only`, or `--save-a`.

For `--nav-to-a`, a fresh check now prints the estimated map pose, distance and
heading error from A, and the relative bearing to A before the Enter prompt.
A goal only a few centimetres behind a forward-only skid-steer robot can
require a large positioning turn; the relative bearing is distinct from the
final heading error. A new freshness check still runs after Enter.

A late localization reader first waits for a bridge-validated parked snapshot,
fresh own odom and scan, the exact scan-time TF and its own passing map fit.
It then starts its private capture-continuity window using the original AMCL
stamp and covariance. Map/TF startup queues before that baseline do not count
as a physical capture gap. After the baseline, capture gaps, motion and clock
changes retain their existing invalidation rules; the reader cannot rearm
this startup allowance within the same session.

The command rejects a missing/legacy/wrong-map A before starting hardware. A fresh whole-map scan search initializes AMCL without importing an old pose; the bridge validates the resulting cloud without requesting a second random global reset. It waits for fresh sensors, converged pose covariance, a concentrated full particle cloud and scan/map endpoint agreement. Ambiguous searches are rejected while the robot stays still. These are consistency checks; they do not establish physical pose accuracy. An explicit initial pose may be supplied in map metres/degrees, or set with RViz:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --nav-to-a --rviz
```

On a headless Pi, use a networked laptop RViz to send `2D Pose Estimate` while the Pi command waits, or supply the measured map pose through `--initial-pose` as shown above.

A single Enter starts Nav2 toward saved A. Nav2 reuses the existing hardware/AMCL and LiDAR obstacle scan. SafeRotationRPP is capped at 0.18 m/s and 0.30 rad/s through the native velocity smoother and collision monitor. StoppedGoalChecker targets 3 cm / 2 degrees and requires low measured linear/angular velocity before success; independent stationary arrival accepts strictly below 5 cm / 2 degrees. `ExactGoalSmac2D` retains Smac2D routing and collision-checks a short connector ending at exact saved A instead of a rounded grid coordinate. A blocked connector is rejected. Fresh covariance and particle spread must remain below 5 cm / 2 degrees with diverse support. These are estimated consistency criteria, not an independent measurement of physical error. Native TF/LiDAR loss stops motion; map/session changes or a latched localization fault cancel the action.

To validate the requested physical tolerance, mark or survey the true base/fork reference at A, repeat arrivals from different starts, and compare measured position and heading with that reference. Keep the physical measurement separate from `/amcl_pose` and Nav2's reported result. The A goal and navigation report retain `physical_alignment_verified=false` until this is independently checked.

A sensor-only localization monitor stays active during Nav2 transit. It publishes no short-trial control odometry and does not impose the A-to-C correction-distance bound on navigation. It checks live AMCL during motion and validates the original AMCL snapshot with live LiDAR/odom while parked. A parked episode gets one refresh; a later refresh requires confirmed travel followed by a stop, rather than periodic resampling. Very short measured movements also rearm the single arrival refresh. At motion onset, the already validated map correction may be projected with fresh odom and newly checked LiDAR for at most two seconds and 8 cm / 0.08 rad of travel, until AMCL reaches its motion-update threshold. This bounded transition is explicitly reported as prediction, preserves the original observation timestamp/covariance, and cannot renew itself or hide a subsequent stale moving AMCL observation. If the fresh current pose is already within the saved A tolerance, no redundant Nav2 goal is sent. After arrival, Nav2 command publishers are stopped while the same localization process remains running. A single role-change service validates fresh stationary evidence and freezes the bounded trial frame at the actual arrived pose. It preserves the same AMCL snapshot, covariance, particles and run identity without another AMCL update or a process restart. Only then are YOLO/RGB-D started. The actual arrived pose, not the nominal saved coordinates, anchors the camera target/path. Enter after viewing camera measurements starts the existing diagnostic A->C trial; insertion and lift remain disabled.

To check the complete localization/Nav2 startup without sending a goal:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --nav-check-only --camera-obstacles
```

The check validates the saved A and map, starts the same precision localization
profile and Nav2 servers, requires the navigation stationary quality window and an
available action server, then writes `navigation_check.json`, stops Nav2 and verifies the same-process
trial handoff into `localization_at_A.json` before cleaning up its owned
processes. It does not enter pallet recognition or A-to-C execution.
Nav2 lifecycle activation completes before the one parked refresh, so server-startup load does not overlap the confidence window. Readiness timeouts report the longest valid window and failed-check counts. Raw scan capture gaps are checked independently of transform/scoring delays; only raw gaps invalidate the snapshot, while delayed scoring requires a new fresh evidence window.
Exact capture-time TF is retried; while a newer scan awaits TF, continuous
navigation may use only an already transformed, independently scored scan
within the existing 0.5-second freshness limit. A newer map mismatch invalidates
that evidence immediately. The continuous readiness fit samples up to 720 real
endpoints to reduce sampling jitter. Runtime recognition and both handoff
checks share >=65% matching endpoints with median wall distance <=5 cm;
teaching a new A retains >=80% / 5 cm. The runtime ratio uses the normal
tracking threshold so returns from unmapped objects do not alone veto an
otherwise accepted stationary pose. It does not replace the independent
<5 cm / 2 degree arrival, covariance, particle-diversity or freshness checks.
Wall distance is the Euclidean distance from the actual endpoint to occupied
cell areas in the rotated map frame, rather than the old quantized distance
between grid indices. Nearest-source candidates always refer to real occupied
cells; missed candidates can only overestimate distance. A sub-micron numeric
margin handles float32 map resolution at the 15 cm inlier boundary.

### Initialize AMCL for a trial already at A

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --global-localization
```

Or use `--rviz` / `--initial-pose X Y YAW_DEG` for explicit initialization. `--map /absolute/map.yaml` selects another map consistently for localization and A validation.

The bridge computes `M(t) = map_base(AMCL stamp) * inverse(odom_base(same stamp))`, freezes `M0` before camera capture and publishes control pose `inverse(M0) * M(t) * raw_odom` in the fixed `r3_trial` frame. Corrections are rate-limited (3 cm/s and 0.5 deg/s), total correction is bounded (20 cm / 5 degrees), and jumps/lost localization stop the trial; no raw-odom fallback is implicit. Target, path, robot control and pallet scan masking all use this same frame. Scan masking interpolates correction history at the scan stamp. Settled correction is required at capture/start/end. The trial bridge's basic AMCL uncertainty gate is 10 cm / 5 degrees std; the separate stationary A gate is below 5 cm / 2 degrees, with 3 cm preferred. With a 5 cm map and no independent position reference, neither estimator covariance nor Nav2 arrival proves physical accuracy. Camera extrinsic bias and small motor steering response still require physical validation.

Logs include AMCL bridge status, scan/map and particle diagnostics, raw wheel/IMU/odom, localized control pose, navigation-to-A result, camera evidence and frozen plan. The result image adds the raw odom trace alongside the AMCL-corrected trace in AMCL mode. No new autonomous hardware execution was performed during implementation.

Map edit 2026-11-12: removed the isolated obsolete obstacle cluster near map (2.26, 1.33) m from `ros2_ws/maps/tai_map_real_edited.pgm` as marked by the operator. 45 occupied cells became free; map dimensions, resolution (5 cm/cell), origin and all other pixels are unchanged. Original PGM and comparison/report are retained locally in `simulation/output/map_edit/`. Because the occupancy-image fingerprint changed, any A taught against the previous map must be explicitly saved again for the edited map. At this edit no `simulation/r3/recognition_A.json` existed.


## RViz on an Ubuntu laptop

### Stationary startup session (2026-11-14)

On the Pi, run this each time you want to start localization. It starts sensors,
AMCL global search, the corrected camera obstacle pipeline, standalone global/local
costmaps and live A-quality checks. It keeps running for laptop RViz. It does not
save A or start a navigation action. There is no OS boot service.

The Pi command can be run directly in a VS Code Remote SSH terminal connected
to `admin@10.181.80.50`. A second `ssh` command is needed only when starting it
from a local laptop terminal. RViz itself runs in a local Ubuntu desktop terminal.

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_localization.sh
```

On the Ubuntu laptop desktop:

```bash
mkdir -p "$HOME/robot_rviz"
scp admin@10.181.80.50:/home/admin/tai_robot_one/robot_amr/ros2_ws/config/r3_laptop.rviz "$HOME/robot_rviz/"
scp admin@10.181.80.50:/home/admin/tai_robot_one/robot_amr/simulation/open_r3_laptop_rviz.sh "$HOME/robot_rviz/"
bash "$HOME/robot_rviz/open_r3_laptop_rviz.sh"
```

If a laptop terminal prints a missing legacy
`/home/tai/tai_robot_one_community/install/setup.bash`, repair that source line
with the helper below. It backs up modified startup files before adding a
file-exists guard; unrelated ROS setup lines are retained.

```bash
scp admin@10.181.80.50:/home/admin/tai_robot_one/robot_amr/simulation/repair_r3_laptop_shell.py "$HOME/robot_rviz/"
python3 "$HOME/robot_rviz/repair_r3_laptop_shell.py"
```

The Pi startup uses system `grep` rather than depending on VS Code's bundled
`rg`. It requires both map server and AMCL lifecycle states to be ACTIVE before
starting the bridge and inspection costmaps. If activation fails, the owned
localization launch is stopped before one retry; a second failure ends the
session with the activation log path. This prevents an aborted lifecycle
startup from appearing to be a running localization session. ACTIVE means the
nodes are functioning, not that their estimated pose is physically trustworthy.

The profile shows live map, both costmaps and footprints, LiDAR, camera obstacle
scan, AMCL particles/uncertainty and separate AMCL/A readiness labels. Corrected
cloud and RGB image are optional displays. The laptop has no drive-goal tool.

In another Pi terminal, inspect the live A-quality result:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/status_r3_localization.sh
```

Only when the operator requests saving A and localization is trustworthy, save
from the current running session without restarting AMCL:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/save_r3_a.sh
```

The A file preserves map X/Y/yaw, full pose quaternion, map-image fingerprint,
AMCL covariance, particle distribution, scan/map agreement, odom pose/transform,
timestamps, stationary duration and recent poses. Publication is atomic and an
existing A cannot be overwritten. The current pose must pass the strict A gate
continuously for two seconds. A collapsed cloud (fewer than ten distinct bins at
1 mm / 0.1 degree, or more than half its mass in one bin) is rejected even if
AMCL reports tiny covariance. These gates still require independent validation
of physical accuracy.

The stricter A scan gate also requires at least 80% of sampled LiDAR endpoints
within 15 cm of mapped walls, and median wall distance at most 5 cm. A basic
AMCL-ready label alone does not satisfy the A-save criteria.

The live camera obstacle cloud uses the same `floor_depth_skew` and
`corrected_depth_m` functions as pallet measurement, evaluated on each current
registered depth frame and calibrated RGB rays. It does not use the older
static `astra_pro_depth_x_skew_preview.yaml` coefficient. Its 4 Hz visualization
publishes the source capture stamp, coefficient and readiness on
`/camera/depth/correction_status`; missing/stale floor evidence withholds the
cloud. The laptop camera scan display expires old points after 0.75 seconds.
Pallet target mounting/range offsets apply to the measured target, not to every
obstacle point. The physical camera TF supplies the cloud mounting transform.

In stationary inspection, `r3_floor_aligned_cloud.py` then checks
the dominant observed ground plane, waits ten seconds for acquisition to settle,
requires at least two seconds of stable fits, and freezes the median plane as
an additional floor-level correction. It keeps the physical
camera XY pivot, accepts at most 6 degrees tilt / 10 cm vertical offset, and
withholds output on missing floor, stale sensors or motion. Both costmaps and
the camera scan use this floor-aligned cloud during inspection. The original
calibration is retained; absolute depth range and RGB/depth registration still
need independent measurements. A transient inconsistent fit withholds camera
obstacles; matching the original frozen plane for two seconds restores output.
Three consistent changed fits spanning a second, robot motion or a changed
camera pivot latch a fault; the frozen transform never adapts in place.
Live evidence is saved as
`camera_floor_alignment.json` in the session log directory.

To stop the session and its owned processes:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh
```

The stop command waits for the recorded session to exit and the real trial
lock to become available before returning success. It also works while global
scan initialization is running. To restart in the same terminal:

```bash
bash /home/admin/tai_robot_one/robot_amr/simulation/stop_r3_localization.sh &&
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_localization.sh
```

An already stopped session returns success when the lock is free. A mismatched
PID is not signaled; cleanup that still holds the lock after the bounded wait
returns an error so the chained start cannot race the previous shutdown.

The standalone costmaps preserve the real footprint/inflation and use LiDAR
plus a separate camera obstacle layer; fork/rear sensor layers are omitted
because those topics are unavailable in this startup flow.

Stationary global initialization uses `r3_global_scan_match.py`: a 20 cm / 5 degree
search over free map cells followed by 2.5 cm / 0.5 degree refinement of distinct
hypotheses. A candidate needs a sufficient score margin over alternative
sampled poses, at least 75% endpoint agreement within 15 cm, median wall distance
at most 5 cm, limited wall crossings, and agreement on held-out beams. A second
fresh scan must independently agree within 5 cm / 2 degrees before a single
`/initialpose` is sent with explicit 5 cm / 2 degree covariance. This is an
initialization bound, separate from the below-5 cm XY RMS / 2 degree A-save
criterion; it is not a physical accuracy result. Motion, stale sensors or changed map content reject
the bootstrap. Reports are stored in `global_scan_initialization.json`.

The stationary AMCL readiness label also rejects collapsed particle support
and scan fit below the inspection 80% / 5 cm thresholds. Odom covariance disks
are explicitly hidden in the laptop profile to remove the large purple disk;
the underlying covariance values and the AMCL pose display are retained.

After the whole-map seed, stationary inspection requests only one startup
AMCL refresh. It does not repeatedly resample the same parked observation.
The original AMCL pose timestamp, full covariance and particle statistics are
retained in the latched localization status. Reusing this snapshot requires
continuous fresh odometry and distinct matching LiDAR scans spanning at least
one second. Late queued messages pause readiness and require a new one-second
window of fresh scans. Motion, gaps in sensor capture timestamps, clock rollback
or changed map content invalidate it until a new AMCL observation arrives.
This check does not freeze TF, fabricate
new AMCL observations, reduce covariance or establish physical accuracy.
The A watcher and save command validate the same evidence even when started
later; both use the user-selected below-5 cm position / 2 degree limits,
with 3 cm preferred. Scan agreement, freshness and full-cloud spread checks
remain in force. New-A teaching also retains its conservative diversity gate.
Their strict 80% / 5 cm scan gate is tracked on every distinct scan for two
continuous seconds when teaching A, including scans between the slower RViz
status reports. The navigation workflow uses a 0.5-second quality hold, as
requested by the operator for the already parked-at-A case; the independent
stationary snapshot still requires at least one second of fresh scan/odom
evidence. Position below 5 cm, heading within 2 degrees, scan agreement,
full-cloud concentration and freshness limits remain. Runtime recognition uses the
shared 65% / 5 cm scan fit rather than the stricter teaching percentage.

Runtime localization, A arrival and the A-to-C handoff do not reject a posterior
solely because fewer than ten quantized pose bins remain or one bin holds over
half the probability. AMCL resampling can produce these concentrated clouds at
a valid pose. Such a cloud must numerically reproduce its original AMCL pose
and covariance even for close publication timestamps; it then passes the same
unimodality, below-5 cm / 2 degree spread, map-fit and fresh-stream checks.
Concentration does not prove physical accuracy. A changed/stale stream, bad
map fit, ambiguous full cloud or excessive spread still prevents acceptance;
no extra parked resampling or synthetic confidence is introduced.

For `--nav-to-a` and `--recognize-here`, the owned `r3_control_odom.py` process
publishes `/pallet/r3/localized_odom` on each fresh EKF capture after the trial
handoff. The supervisor is the only owner of AMCL validation and bounded
correction updates; it no longer publishes that odom topic in this mode.
Map/particle validation can therefore delay its own queue without withholding
fresh EKF captures from the controller. EKF capture freshness remains 0.2 s.
Initial reference acquisition retains the supervisor's 0.5 s lease and the
existing AMCL/scan/full-cloud checks. As requested by the operator, once that
reference exists, delayed or rejected AMCL updates leave the last validated
correction unchanged and EKF tracking continues. The 2 s AMCL / 0.5 s scan
limits still gate adoption of a new correction, not continuation of a locked
A-to-C path. The control status explicitly reports `ekf_tracking` and the
original AMCL source age; it never invents a fresh AMCL observation. Fresh
AMCL updates may resume bounded correction in the same frozen frame.
Actual EKF loss/jumps/clock reversal, a different map/run/baseline, loss of
control integrity, IMU loss and collision safety still stop the robot. Normal
goal completion still stops at C. Original sensor stamps/covariance bounds
are preserved, and the odometry publisher never resets a frame or calls AMCL.
The supervisor defers only its own queued-EKF freshness check to this process
in the trial role. Native Nav2 monitoring keeps its existing rules.
`control_odom.log`, `/pallet/r3/control_odom/status`, supervisor processing times
and the trial's localized-odom capture/receive diagnostics expose delays.
The frozen plan records `control_status_topic`. The follower and pallet
obstacle filter consume the same independent control status/correction history,
so an AMCL supervisor delay cannot also starve obstacle projection. Capture
still requires the original localization quality and freezes the publisher's
actual applied correction. No extra parked AMCL resampling is requested.

The A-to-C follower keeps one persistent executor and retains only the latest
sample per sensor subscription. It services queued inputs before each 20 Hz
control decision instead of letting an overdue timer check an old receive cache.
After a processing delay, fresh captures can therefore resume the same frozen
path; actual missing or stale source captures still fail the unchanged 0.2 s
odom/IMU guards. Diagnostics include tick intervals, execution/publish times and
input-drain counts under `control_loop` in `odom_imu_diagnostics.jsonl`.

`r3_receiver_ros_regression.py` exercises the production follower and recorded
93-pose plan with a separate 50 Hz source process in LOCALHOST ROS domain 231.
An injected 400 ms receiver pause reproduces the timer-based false stop; the
new loop drains the queued fresh captures and continues. Separate old-capture,
stream-loss and collision cases still stop. Results are stored under
`simulation/output/control_receiver_ros_regression/`; this is a transport and
control regression, not a physical robot run.

The trial limits YOLO acquisition to 1 FPS while keeping RGB-D tracking at
8 FPS, the same FP32 model and the same measurement calibration. The failed
run's roughly 750 ms inference calls exceeded the old 2 FPS period, causing
back-to-back inference during repeated tracker reacquisition. The 1 FPS limit
leaves CPU time between calls. A camera-only check stopped YOLO after 5.92 s,
then collected five valid post-off captures over 0.532 s without tracking-gap
warnings; its report is `simulation/output/camera_latency_probe_1fps/report.json`.
This check started no wheel controllers and does not establish full-stack
navigation/acquisition timing or absolute camera accuracy.

The stationary inspection AMCL profile uses `likelihood_field_prob` with beam skipping,
120 beams, a 10 cm hit likelihood width and 2,000--4,000 particles. This replaces
the weak default additive likelihood update; it is not a calibration of the
map or proof of absolute pose accuracy. The initial global-search covariance
remains 5 cm per axis / 2 degrees. The single startup refresh waits until both
pose and particle-cloud interfaces have matched for one second and exact
odom TF history covers the full two-second observation-age limit plus the
current scan. AMCL stamps
its pose at scan capture and its cloud at publication; a lag beyond 250 ms
requires the full cloud mean and covariance to reproduce the original AMCL
posterior within tight numerical bounds. Both stamps and that pairing evidence
are retained for late A readers.

The bridge also retains up to 12 recent original particle clouds. If a pose
was delayed waiting for its capture-time TF and the latest cloud describes a
later posterior, it may use its retained matching cloud only after checking
the full numerical mean and covariance, even when the timestamps are close.
The two-second freshness limit and the 8 cm / 0.08 rad motion-onset limits are
unchanged. No observation stamp is replaced by its delivery time.

During travel to A, commands use the native chain
`/cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_safe`.
AMCL continuously supplies map-to-odom TF and EKF supplies odom-to-base TF.
The native controller/costmaps check TF health, footprint collision checking
and the LiDAR collision monitor remain active, including its 0.5 s sensor
timeout. A Python quality report is not an interlock in this command chain.
The experimental onset gate is not launched: its two-second stop caused
`104055_abEAD1` to abort for lack of progress, and repeated report-based
stops made `104300_vOiYSw` move intermittently before its client canceled.

The action client keeps a registered executor and checks map/session/role
identity plus latched localization faults and large validated map-correction
jumps during the same Nav2 goal. Independent stationary scan/particle quality
checks bracket this action; repeated parked scan scoring and a duplicate
particle-cloud subscription are omitted during native travel. The bridge's
display readiness may pause without interrupting healthy native TF navigation.
Recognition A is a coarse region below 10 cm / within 5 degrees, as requested
by the operator. If already inside this region, recognition starts without
sending a redundant goal. Estimator spread/stability remains below 5 cm /
2 degrees; the saved A pose and RGB-D geometry are unchanged.
The role handoff still requires all Nav2 velocity publishers to disappear.

The Nav2 StoppedGoalChecker targets 8 cm / 4 degrees and requires measured
rotation <=0.02 rad/s and translation <=0.015 m/s before success. Cruise is
capped at 0.18 m/s and heading alignment at 0.30 rad/s;
the approach minimum is 0.10 m/s, above the loaded drivetrain's low-speed
stall region. The existing RPP loaded angular floor is 0.22 rad/s. Braking
uses the real drive's configured 0.5 m/s² and 1.5 rad/s² limits.
Precision turns near the endpoint are capped at 0.22 rad/s and brake early
using measured angular speed, the configured 1.5 rad/s² deceleration and a
0.25 s delivery/response budget estimated from the latest physical overshoots.
A zero-command hold waits for low measured motion before another correction;
it does not immediately reverse direction after crossing the heading target.
This behavior is opt-in for the A profile; other controller profiles keep
their existing behavior.

The A profile also permits a new straight rear correction no farther than
12 cm, with tight lateral/heading bounds. The complete corridor is checked
with the physical footprint and native collision prediction. Reverse speed
is limited to 0.10 m/s; the downstream smoother and reverse stop envelope
cover that range. Longer reverse-route tracking is disabled in this profile.
The stationary-turn budget and all LiDAR collision checks remain active.
A passive recorder writes `navigation_motion.jsonl` during travel to A,
including requested/smoothed/safe/drive-limited commands, raw gyro and odom.
It publishes no commands and stops before the A-to-C role handoff.


After native success, stationary verification has up to 20 seconds within the
original action deadline. There is at most one native goal attempt; automatic
repeated final position/yaw corrections are disabled. The measured terminal
pose must lie in the coarse 10 cm / 5 degree recognition region.
When XY is already inside the allowed A region and only yaw needs correction,
the temporary native goal keeps the measured current XY and requests saved A's
yaw. This avoids a short positioning loop; final acceptance is still measured
against the original saved A, and the A file is never changed. The same rule
applies at startup when the robot is already in A's allowed position region.
`arrival_attempts` preserves each measured terminal pose and its evidence.
Timeout diagnostics retain the last actual stationary candidate and errors to
A, while `navigation_result` records native success separately from acceptance.

A capture gap before trial-frame freezing may request one genuine new AMCL
observation after stationary, fresh odom/scan and exact TF remain healthy for
one second. The old source remains invalid until the new pose/cloud pair is
accepted. This is not periodic resampling; reverse clocks, changed maps and
the frozen C trial frame cannot use this recovery path.

Live scan checks also report robot-left and rear-left endpoint regions, using
the complete LiDAR mounting transform. These diagnostics correspond to the
stable boundary and corner identified by the user. They do not move the pose,
change scan/map acceptance thresholds or replace global map geometry. Its
likelihood overrides apply to `--localization-only`, `--recognize-here` and
`--nav-to-a` (including their C motion), using the
`precision_localization` launch argument. The
bare existing trial retains its earlier profile. The pallet camera measurement
is unchanged.

Use Ubuntu 24.04 / ROS 2 Jazzy on the laptop and the same LAN as the Pi. Both machines must use the same ROS domain (commands below select 0), compatible middleware and SUBNET discovery. RViz runs on the laptop, not inside the Pi SSH session. No robot package build is needed on the laptop for this primitive-geometry URDF; `/robot_description` and TF arrive from the Pi.

Pi SSH terminal, localization/view only (no Nav2 or A->C motion); Astra obstacle visualization is also started without YOLO:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
bash /home/admin/tai_robot_one/robot_amr/simulation/run_r3_live_trial.sh --localization-only --global-localization
```

The map and particle cloud can be viewed while AMCL is still searching. The command keeps localization alive until Ctrl+C. If global localization remains ambiguous, the laptop's `2D Pose Estimate` tool can give the current approximate pose; it does not save or change recognition A.

Laptop terminal (replace the Pi IP if its address changes; observed on 2026-11-12: 10.181.80.50):

```bash
mkdir -p ~/robot_rviz
scp admin@10.181.80.50:/home/admin/tai_robot_one/robot_amr/ros2_ws/config/r3_laptop.rviz ~/robot_rviz/r3_laptop.rviz
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
rviz2 -d ~/robot_rviz/r3_laptop.rviz --ros-args -p use_sim_time:=false
```

The odometry displays keep only one arrow and use zero position/angle display
tolerances, so a new stationary message also replaces the old arrow after a
Pi session restart. RViz runs independently on the laptop: stopping the Pi does
not clear its display history. Download this updated profile and reopen RViz,
or click **Reset** to clear the current laptop display immediately.

Laptop visualization dependencies, if missing from an existing Jazzy installation:

```bash
sudo apt update
sudo apt install -y ros-jazzy-rviz2 ros-jazzy-nav2-rviz-plugins ros-jazzy-rmw-fastrtps-cpp
```

The laptop profile uses direct topics: `/map`, `/robot_description`, `/scan_filtered`, `/amcl_pose`, `/particle_cloud`, `/odom`, `/plan`, `/received_global_plan`, `/lookahead_point`, global/local costmaps and footprints. It includes the Navigation 2 panel, Nav2 Goal tool and costmap cost tool. The owned lifecycle managers use the standard Nav2 names so the panel can query their state. The dark-yellow `/camera/obstacle_scan` display is enabled in localization-only mode; corrected cloud is optional to save bandwidth. Both costmaps now publish during stationary localization; plan/lookahead displays populate only during navigation. Keep the running session to save A with `save_r3_a.sh`; stop it before starting a separate trial/navigation session. Do not run two hardware/AMCL sessions simultaneously.


Global AMCL diagnostic (2026-11-12): the first stationary global-search trial collapsed to a wrong narrow hypothesis (~3.184m,1.607m,-87.27deg). Only 61.67% of sampled endpoints matched within15cm, below the65% acceptancegate. This was correctly blocked; repeated stationary no-motion updates can deplete particles and narrowcovariance is not independentaccuracy. The bridge now preserves/reports rejected fresh-candidate scan/map fit even before any acceptedbaseline exists, and publishes a green/red readiness text marker to RViz. A `2D Pose Estimate` seed of the actual current robot pose can recover this case without changing recognitionA. Automatic stationary global localization is not validated as reliable in this room; do not treat the visible robot in RViz or GlobalStatusOK as localizationready.

Navigation handoff fix (2026-11-14): the accepted A pose in the failed session
`162649_3NmWhF` was already inside the 5 cm / 2 degree limits. Restarting the
localization monitor caused an unnecessary extra AMCL parked update and particle
collapse. `r3_enable_trial.py` now waits for Nav2 velocity publishers to disappear,
enables `/pallet/r3/localization/enable_trial` and verifies the unchanged
snapshot plus a fresh stream of bounded control odometry. The service never
reanchors a second time. The saved A coordinates are unchanged.

The role-change client waits for strict fresh source quality before requesting
the handoff. An explicit rejected response with unchanged healthy source may
be retried up to three attempts, at least 0.25 s apart within the same 30 s
deadline. Uncertain responses and source/role/fault changes are not retried.

Laptop RViz now includes `nav2_rviz_plugins/Navigation 2`, `GoalTool` and
`CostmapCostTool`. Reload the copied profile after closing the previous RViz
window. The lifecycle manager names follow Nav2's standard names so the panel
can query the actual localization/navigation state. After reaching A, Nav2
servers stop before the separate A-to-C follower owns the drive command path.
