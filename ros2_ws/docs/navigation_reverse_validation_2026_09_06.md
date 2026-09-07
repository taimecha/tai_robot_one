# Checked reverse recovery — 2026-09-06

## Changes

- Keep forward-only RPP path following (`allow_reversing: false`). After an
  alternative SE2 execution and checked forward escape fail, try BackUp for
  0.15 m at 0.05 m/s, with an 8 s allowance, then plan and execute again.
  The existing outer retry budget bounds repeated recovery attempts.
- Allow negative X velocity down to -0.08 m/s in the smoother. Add a reverse
  stop polygon covering the padded body and extending to x=-0.50 m behind it.
- Add a simulation-only rear low scanner at x=-0.34 m, z=0.12 m facing backward.
  Bridge `/rear_scan` and require it in Collision Monitor; include its obstacle
  layer in the local costmap. Hardware needs equivalent rear sensing before
  using this configuration. The sensor is a single plane, not full 3D coverage.
- Change local resolution from 5 cm to 2 cm, preserving footprint and padding.
  The grid has 6.25 times as many cells. Global lattice resolution stays 5 cm.
- Reduce full rotation sweep sampling from 0.02 to 0.01 rad for the finer grid.
- Add optional `spawn_x`, `spawn_y`, `spawn_yaw` launch arguments, with matching
  AMCL initial pose. All defaults remain zero. The original launch command works.
- Preserve the user's current 0.30 m/s translation, 0.30 rad/s alignment speed,
  0.10 m XY tolerance and 5 degree yaw tolerance.

## Diagnosis

At the recorded wall pose, a padded footprint edge lay in a lethal 5 cm cell
although current lidar returns were approximately 1.9 cm farther toward the
wall in odom X. Whole-sweep checks and native BackUp both check their initial
pose, so a collision there rejects both rotations and translation immediately.
No checks were disabled and no obstacle layers were cleared as recovery.

An initial test teleported the robot into an already-running map and BackUp
returned collision error 714. That test does not isolate the stationary startup
case; subsequent tests spawn directly at the recorded Gazebo pose instead.

## Validation

Build succeeds. Targeted CTest suites `test_rotation_sweep` and
`test_stop_envelope` pass; the latter now also verifies the reverse polygon
contains the padded body and protects the rear stopping margin.
Xacro expands the rear scanner frame and plugin successfully.

Headless run log:
`/home/tai/.ros/log/2026-09-06-15-54-53-233992-DESKTOP-B3BME4V-2003`

Spawn: (-1.6314285, 1.9168707, yaw=2.8164593).

| Leg | Action result | Wall seconds | Feedback recoveries |
| --- | --- | --- | --- |
| Wall pose to (0,0,0) | SUCCEEDED | 35.25 | 0 |
| Origin to (1.532,-1.412,-0.012) | SUCCEEDED | 17.85 | 0 |
| Return to origin | SUCCEEDED | 33.56 | 0 |

These navigation runs used the previous 0.02 rad sweep sampling; the final
0.01 rad sampling has a separate GUI retest. Feedback counters alone do not
prove absence of recovery actions, but behavior logs also show none in these legs.
TF sampled after result delivery is asynchronous and is not an independent
measurement of final physical accuracy.

Direct BackUp action in clear space: SUCCEEDED, 3.19 s simulation elapsed,
0.1530 m odom displacement. A temporary static box (10 x 40 x 20 cm) behind the
robot produced rear scan returns at 0.1073 m; BackUp returned collision error
714 immediately, with 0.0 m odom displacement. The test box was removed.

This validates clear-space reverse motion and refusal of a low rear obstacle,
not every recovery situation or deployment safety. Native BackUp still refuses
an initially colliding footprint; the finer grid is not permission to escape
through a real obstacle. Full autonomous BT reverse triggering needs broader
scenario coverage; the wall navigation run escaped without needing BackUp.

Final GUI retest with 0.01 rad sampling:
`/home/tai/.ros/log/2026-09-06-15-59-27-250178-DESKTOP-B3BME4V-4093`.
Same wall spawn to origin SUCCEEDED in 63.91 wall seconds. Behavior logs show
two successful forward escape actions; no BackUp was needed in this leg.
Feedback reported zero recoveries, demonstrating why logs must also be read.
The GUI run is slower and not evidence of globally optimal navigation.
Gazebo and RViz are left running at the end for the user's next test.
