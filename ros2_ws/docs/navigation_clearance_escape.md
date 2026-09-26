# Real navigation: footprint clearance and continuous escape

`nav_real.launch.py` selects `navigate_real_clearance_escape.xml` in normal
mode. `motion_test:=true` keeps its separate diagnostic tree. The real-only
VelocityStop override remains disabled; FootprintApproach and controller /
BackUp collision checks remain enabled. Simulation keeps its existing tree.

For a nearby goal (at most 75 cm, within 0.10 rad of forward), the real tree
first tries a dense direct route from the actual pose to the exact requested
goal. It checks the entire padded body, launch sweep and shortest terminal
sweep against both costmaps; no initial map overlap is ignored. If feasible,
its straight translation is the shortest XY route and avoids cell-grid kinks.
For other goals / blocked direct routes the tree requests GridBased. A custom
PathFootprintClear BT condition validates the actual published padded body /
fork polygon along that path and at the initial and final heading sweeps.
SE2Fallback searches orientation-aware alternatives when the XY route does
not fit. A valid current path is retained instead of replaced every 2 seconds.
This is a preference for short feasible routes, not a proof of global optimality.

The condition uses both global and local raw costmaps near the robot, and the
global costmap farther away. Samples are at most half a cell / 1 cm in combined
translation and fork-tip angular travel. Occupied edges, enclosed obstacle
cells, unknown cells and footprints outside the map are rejected. Initial /
terminal rotations use only the shortest correction; initial rotations may
try the other complete sweep. Recovery evaluates possible turns rather than intentionally rotating
until contact or an emergency stop.

After failure, recovery attempts a validated SE2 path and then a fresh
validated GridBased path. It next searches complete routes from checked
forward starts before falling back to local manoeuvres. A distant footprint/
terminal failure or occupied goal does NOT authorize unchecked local recovery
motion. Start-occupied, near-body path
collision and failed control/progress can authorize local escape; terminal
alignment within 10 cm of the goal cannot. Error state is reset at the start
of a new tree execution and before the recovery alternatives.

## Ranked complete forward routes

`ForwardRouteSearch` runs before the old restricted local escape heuristics.
While stopped, it proposes starts at 0.15, 0.30, ..., 1.20 m straight ahead,
bounded by remaining goal distance and the fresh full-body global/local
departure corridor. For each proposal the native ComputePathToPose action
uses the explicit hypothetical start (`use_start=true`), first GridBased,
then SE2Fallback if the complete GridBased route fails validation. The full
departure from the actual current pose is prepended at 2.5 cm intervals;
any quantized launch-heading correction is placed AT the future start.
The entire joined route, including every rotation primitive and the exact
requested terminal pose, must pass PathFootprintClear before it is retained.
Local observations cover the whole departure prefix even beyond one metre.
Proposal checks issue no velocity or navigation actions.

Retained candidates are ranked by path length plus 0.10 m per radian of total
heading variation and 0.05 m per stop-to-turn manoeuvre. This strongly prefers
short routes and fewer/smaller turns; it is not global shortest-path optimality
or an exhaustive search of every possible position, heading or motion.
The selected route is revalidated against current pose/maps before execution,
and a reactive route/data guard continues checking while FollowPath runs.
When it becomes blocked, or control reports patience/no-progress/no-valid-
control, the next ranked route is revalidated from the new pose before it can
run. Invalid alternatives are skipped, not executed. A running route stays
selected; searching does not restart every controller tick. A changed goal or
halt cancels the current planner/action and clears the search choices.

Only after the finite forward route choices are exhausted does the existing
checked local forward-pocket/turn/continuous-retreat fallback run. Full routes
may be searched after distant path-validation failure because they prove the
entire connection to the goal; distant failure still cannot authorize blind
local escape. Controller unknown/invalid-controller/TF/invalid-path/timeout
faults stop route retries and cannot authorize subsequent local motion even
if the planner reported Start occupied. Stale sensors/maps/TF still stop
movement. A 60 s stopped planning deadline prevents a hung planner waiting
indefinitely; after it, only previously validated and freshly revalidated
candidates may execute. Failure/abort can still occur for unreachable goals,
faults, or exhaustion of this bounded candidate set.

Local escape first checks whether a useful turn is available at the current
pose. Otherwise it searches forward turning/exit pockets at 15 cm intervals,
up to 1.20 m and bounded by remaining goal distance. Clear translation alone
is no longer sufficient: the destination must also permit a checked turn and
30 cm forward exit, or a goal-directed straight corridor up to 75 cm. All
intervening translations, turn sweeps and exits use the unmodified global AND
fresh local maps (including beyond one metre). Both signed turn directions
are evaluated at 15-degree candidates plus the exact goal bearing; heading
alignment is preferred. During the first local attempts an angled detour may end retreat without the
entire goal-facing sweep fitting: its complete selected turn AND a 30 cm
forward exit must fit. A bare partial turn without a checked exit still fails.
After braking, that selected turn executes immediately and the full route
is replanned from the resulting pose; this local exit is not proof of arrival.
Turning-pocket checks add a private 4 cm footprint margin and
0.07 rad braking margin; the published footprint and rear translation
footprint are not altered. Tiny turns still pointing into the dead end are
not considered progress. This is local geometric
escape selection, not proof that an entire route to the destination exists.
DriveOnHeading is reactively guarded over the remaining translation and
destination turning pocket. A lost pocket/corridor, collision result 723, or
successful advance with no remaining exit switches to checked retreat rather
than aborting immediately. A stopped 0.3 s Wait and fresh-data gate separate
the canceled forward action from reverse, so opposing behavior commands do
not overlap during direction changes. Drive/Spin timeout, TF, invalid-input and unknown
faults still propagate instead of authorizing movement after a system fault.
Spin COLLISION_AHEAD (703) instead rechecks the fresh full-body 16 cm rear
corridor. If safe, the SAME navigation goal continues with continuous retreat;
if blocked or stale it stops safely. Before another recovery turn it requires
another freshly checked useful exit with margin. The failed turn heading is
excluded within 15 cm of its recorded odometry launch location, but another
safe heading can execute immediately, without an arbitrary extra retreat.
If a Spin collision has no recorded attempted heading, the conservative
15 cm measured retreat is still required. A geometrically failed forward
escape immediately commits to seeking different space, rather than choosing
another tiny partial turn in the same pocket (see region memory below).
Turning in place, advancing, or an AMCL correction does not count as reverse
travel. Ordinary replanning resets do not erase this commitment; a new
navigation execution/goal does.

## Region memory and lost turning pockets

Local manoeuvres do not prove a complete connection to the goal. Record
executed/attempted local turns and completed local advances in a goal-scoped
odometry region. After two such adjustments within 45 cm, stop proposing
another short local advance or partial turn there. A geometrically stopped
advance commits immediately. Ordinary ResetEscapeState, path invalidation
and a successful partial Spin preserve this memory; a new goal/execution
clears it. Complete normal Direct/GridBased/SE2 routes remain eligible and
fully collision-checked; merely seeing a planner path flash is not evidence
of an executable body-safe route.

During commitment, a retreat can still end immediately if the entire selected
goal-facing turn, private margins and its forward departure corridor fit at
the CURRENT pose. A 15-degree partial turn alone cannot interrupt it. A
successful goal-facing Spin releases commitment; otherwise 30 cm signed
odometry retreat with at least 25 cm net displacement releases the regional
restriction so alternatives can be reconsidered. These are hysteresis
thresholds, not blind distances: rear obstruction, native collision checks
or stale data stop reverse first. Pure forward/sideways travel and rotation
do not satisfy the reverse threshold. This finite heuristic is not an
exhaustive search or a guarantee that no physical route exists after abort.

If a turning pocket disappears during the stopped braking handoff, or the
selected Spin fails its fresh geometric launch check, `lost_exit` allows one
fresh planning cycle and further checked retreat instead of immediate abort.
It requires an actual pending handoff/geometric launch failure, error code
zero, fresh data and a full-body safe rear corridor. It is consumed once.
Spin timeout, TF, invalid-input and unknown faults cannot use this allowance;
BackUp faults without a turning handoff cannot use it either.

Only with no forward/turn exit is BackUp allowed. It stays active at 0.08 m/s,
not restarted every 5 cm. A reactive guard checks clearance at 5 Hz and halts
BackUp immediately when a useful exit exists at the CURRENT pose, or the full-body
16 cm rear corridor is obstructed. All original behavior-server collision
checks, velocity smoothing and Collision Monitor remain downstream. The
10 m / 180 s action limits are fault bounds, not a blind requested retreat;
the guard normally cancels much earlier. Missing/stale sensor/TF data or a
non-geometric motion-action fault terminate recovery safely. Rear blockage
stops reverse motion, but after measured retreat the tree first requests one
new planning cycle from that stopping pose. Arrival at an
unreachable goal is not guaranteed, and a stalled motor is not geometric proof
that no manoeuvre exists.

While reversing, an open short forward segment or a hypothetical pocket ahead
does NOT interrupt the action and send the vehicle back into the old dead end.
The same BackUp goal remains active while retreating toward a turning pocket.
After a usable exit is reached, BackUp is halted, a 0.3 s stopped Wait permits
braking, fresh data and the turn are rechecked, and Spin executes before
replanning. A failed Spin geometry check commits another measured retreat;
other faults stop safely. Executed turn headings are remembered in odom within
15 cm of their launch location, so the same local turn is not retried endlessly;
translation or a new goal permits reconsideration. DriveOnHeading
has a 25 s fault deadline to accommodate the longer checked forward search.
Simulation, footprint dimensions and the downstream safety chain are unchanged.

A retreat can reach a pose from which a compound SE2 route is feasible even
though the stricter local goal-facing turning-pocket heuristic still rejects
it. `replan_after_retreat` prevents treating that rejection as proof that no
route exists. When the reactive guard geometrically stops a retreat, or
BackUp reports COLLISION_AHEAD (714), at least 5 cm signed reverse displacement
in odom permits exactly one extra planning cycle. The previous path is
invalidated, a 0.3 s stopped Wait allows braking/new observations, and fresh
data are required again. Normal Direct/GridBased/SE2 planning and recovery
then run from the new actual pose, including checked forward pockets if the
new route still does not fit. Each complete candidate route still passes
the full published-footprint validator before FollowPath can execute it.
This does NOT relax the private turning-pocket margin or accept an unchecked
partial turn: local angled exits must include a checked forward corridor,
and every subsequent complete route still requires full validation.
No periodic restart is added to the running BackUp action. The replan allowance
is consumed once, is cleared for a new goal/execution, and cannot be earned
by standing still, moving forward or an AMCL pose jump. BackUp timeout, TF,
invalid input and unknown faults never authorize it; stale data stop it too.

## Narrow padding-only retreat release

A global-only overlap confined to the extra 3 cm padding no longer necessarily
locks reverse recovery. A private validation map can omit initial lethal cells
in that padding only if the unmodified core body and the entire live local body
are clear. Unknown cells, core collisions, newly encountered global obstacles
and any local observation remain blocking. The whole 30 cm rear release must
fit and its endpoint must fit the unmodified global/local footprint. Normal
rear guarding still checks 16 cm continuously; no published map, physical
footprint, native BackUp collision check or Collision Monitor is disabled.
If the static map overlaps the core, navigation still stops; this is not a
general waiver for Start occupied or evidence that an obstacle is fictitious.

SafeRotationRPP supports the opposite complete sweep away from the goal and a 378-degree
stationary-turn budget. A measured 4 cm translation clears that budget, including
after it was tripped. Final XY acceptance is latched while aligning heading;
final turn tolerance is taken from the goal checker instead of hardcoded 2.6
degrees. Final alignment recomputes the signed shortest angle every cycle and
brakes before reversing angular direction. Goal acceptance remains 5 cm / 5 degrees.
Within 50 cm of the endpoint, alignment never selects a long opposite sweep;
a blocked short turn requires another checked manoeuvre instead of a full lap.

Tracking uses the immediate path segment for initial heading rather than the
far lookahead carrot. A lattice rotate-in-place primitive limits lookahead:
the vehicle advances to that position before turning. Translation retains RPP
speed / cost regulation and collision checks, and tries shorter carrots when
the longer tracking arc would collide. This avoids rejecting a planned
"advance 15 cm, then turn" route by trying to turn at the original position.

GridBased XY path orientations are reconstructed from segment tangents and
the requested terminal goal pose before validation AND controller execution.
The terminal XY is also restored to the actual requested goal, not its
discretized map-cell corner; this exact endpoint must still pass collision checks.
Very short Smac2D plans can otherwise contain identity intermediate yaw and
ask a west-facing robot to spin almost 180 degrees at the destination.
Lattice SE2 orientations and rotation primitives are never rewritten.
Raw Jazzy Smac2D intermediate cell corners are converted to physical cell
centres once. A retained prepared route is never shifted again. This removes
a half-cell start bias that can make a short straight route request a large
initial turn while the geometry validator assumes a nearly straight departure.
When within 10 cm of the route, controller heading alignment uses its first
travel-segment tangent rather than a biased tiny cell-to-robot bearing.
An upcoming in-place turn still limits lookahead and is not anticipated early;
large off-route offsets retain point-bearing alignment.

The BT condition composes current odom->base with the latest map->odom.
It requires fresh odometry, costmaps and footprint; the timestamp of a held
stationary AMCL transform is not incorrectly treated as stale odometry.
Private condition nodes ignore process-wide ROS node-name remaps.
All conditions now share one costmap/footprint subscription cache per tree.
An asynchronous readiness step waits up to two seconds for initial callbacks
and matching TF, with no motion command, instead of aborting a new goal on its
first tick. Once motion has begun, stale data still stops recovery immediately.

Forward departure may ignore an existing global raster overlap confined to
the extra 3 cm padding when the complete live local footprint is clear.
This affects only a private validation copy: core-body cells, unknown cells,
observed local obstacles and newly encountered map obstacles remain blocked.
It handles a stationary vehicle touching a map cell at the padded rear edge
without disabling obstacle layers or altering the published footprint.

A separate explicitly checked local escape can also depart a map-only overlap
of the starting body: only the initially intersected lethal map cells are
masked in a private scratch copy for that forward translation; unknown and
newly intersected cells remain blocked. Every sampled local footprint must
fit, and the endpoint must fit the unmodified global map. Normal path
validation and reverse/turn sweeps do NOT ignore core-body map obstacles.
This handles Start occupied without deleting map data or inventing a new pose.
It does not establish that localization is correct or that LiDAR sees low /
occluded obstacles; the operator must verify those physical limitations.

Tests run in isolated ROS domain 231, never issue velocity commands, and cover
clear and narrow corridors, an obstacle enclosed inside the fork footprint,
stale maps, held localization with fresh odometry, advance-before-turn,
short final-angle correction, interpolated motion and turn-budget recovery.
The real tree is loaded against installed Nav2 port declarations with motion
actions replaced by test builders. Loaded-floor behaviour and obstacle / fork
clearance still require observation on the actual robot after restart.
Regression tests also cover rejecting recovery for distant obstacles,
preferring a viable turn to reverse, retaining one continuous backup action
and halting it when an exit appears, and clearing old goal error codes.
Additional regression scenarios include a clear short forward dead end with
a rear turning pocket (one continuous retreat until the pocket), a forward
pocket beyond an arbitrary 30 cm step, and a newly blocked rear corridor.
Spin-collision fallback tests cover continuous additional retreat rather than
immediate abort/repeating the same turn, checking the whole SELECTED sweep and
forward exit rather than requiring the goal-facing sweep, accepting a different
checked heading without arbitrary extra retreat, and refusing reverse after
timeout/TF faults. The actual autoremap retreat subtree is executed with mock
native actions: one BackUp is halted and one Spin executes as soon as an angled
pocket becomes available. Other tests cover a lost forward corridor switching
to continuous BackUp, failed-advance fault rejection/reset, and narrowly
allowing map-only padding release while rejecting core/local/unknown contacts
or a new rear obstacle.
Retreat-stop tests additionally verify one halted continuous BackUp followed
by a new planning attempt, consuming the allowance once, invalidating the
old route, permitting geometric collision replanning, and rejecting timeout/
TF faults, no displacement, forward travel, localization jumps and old goals.

`scripts/inspect_navigation_clearance` captures current costmaps / footprint /
TF and requests paths from hypothetical starts, without sending navigation
or velocity actions. Captures can be replayed in the isolated BT tests with
`TAI_CLEARANCE_CAPTURE=/absolute/path/to/capture.json`. The diagnostic probes
use an explicitly supplied goal yaw; RViz screenshot goal yaw is not assumed.
`TAI_CLEARANCE_ADVANCE=-0.20` can replay a hypothetical rearward start from
the capture (and its matching SE2 probe) without moving the real vehicle.
`TAI_CLEARANCE_MOTION=turn` / `reverse_needed` select local escape queries;
`TAI_CLEARANCE_EXPECT_FAILURE=1` selects a deliberately blocked expectation.
`TAI_CLEARANCE_MOTION=replan_after_retreat` replays a synthetic 15 cm retreat
to the capture's stopping pose, checks that the rear guard stops there, and
verifies the one-shot replan allowance without moving the real robot.
`TAI_CLEARANCE_MOTION=forward_route TAI_CLEARANCE_ADVANCE=0.15` keeps the robot
at the captured actual pose and validates the full joined departure plus
the matching hypothetical-start SE2 route. It does not move the robot.

`scripts/test_real_short_goal` is an explicitly authorized REAL motion test,
not a read-only probe. By default it sends one short forward goal with unchanged yaw
and cancels on stale sensors, excess travel, reverse, large rotation or timeout.
Use `python3 scripts/test_real_short_goal --distance 0.45` only with a clear
physical corridor and an operator ready to stop the robot. Cancel/stop uses
normal Nav2 and `/cmd_vel_nav`, never bypassing the safety chain.
`--heading` can supply an explicit map heading, and zero distance is supported
for a pure terminal-heading test; the same 35-degree rotation watchdog remains.

## Live check, 2026-09-26

With the edited real map, EKF and filtered LiDAR, the updated stack accepted
two consecutive short real goals after the earlier aborts. The 25 cm forward
goal succeeded with zero recoveries and 4.3 cm localization-based XY error.
The next 20 cm goal succeeded with one checked 15 cm forward escape and 2.4 cm
XY error. Neither test reversed or exceeded its rotation watchdog. The robot
was stopped afterwards. These are map/TF errors, not independent physical
accuracy measurements. Continuous BackUp cancellation is covered by the
isolated running-action regression, but a long real reverse was not tested.

## Stationary spin-collision regression, 2026-09-26

The operator's later goal ended after a checked retreat followed by a -0.52 rad
Spin: behavior_server reported COLLISION_AHEAD and the previous tree propagated
that directly into navigation abort. The updated tree instead checks the rear
and continues the same goal with measured additional retreat. Tests use mocked
motion actions in domain 231; no real goal was issued for this correction.

A stationary capture at approximately (-0.371, -0.230), heading 62.93 degrees,
was replayed with the last reported goal XY (3.40, 1.40) and an explicitly
hypothetical terminal yaw of zero (the RViz goal yaw was not captured).
Turning-pocket selection rejects the original pose and a 10 cm hypothetical
retreat, but accepts a complete useful turn/exit after a 20 cm hypothetical
retreat. This demonstrates local clearance, not execution success or proof
for the operator's exact terminal orientation. The physical vehicle stayed
stationary throughout these probes.

## Stopped-retreat replanning regression, 2026-09-26

A later live goal toward (3.32, -0.07) backed up for approximately 8 seconds
before the BT canceled BackUp and aborted, without a BackUp timeout or
collision result. A subsequent stationary capture near (0.067, -0.276),
heading -150.19 degrees, rejected local turn/forward escape and another
16 cm retreat, while a SE2 route passed full-footprint validation with an
explicitly hypothetical terminal yaw of zero.

The updated isolated replay verifies that a synthetic 15 cm measured retreat
to that captured pose now earns one stopped replan instead of immediate
abort. The SE2 route still passes the original validator. All three selected
test groups (stop envelope, rotation sweep, footprint BT) pass; the optional
capture replay was also run separately for both route validation and the
new one-shot allowance. These are no-motion tests, not proof of success for
the operator's original terminal heading. The live stack was not restarted
and no real navigation goal was sent for this change; the operator requested
restart commands to apply it themselves.

## Complete forward-route regression, 2026-09-26

After restart, two further goals still backed up and aborted. Logs showed
controller rotation collision stops, followed by the new stopped-retreat
replanning allowance; that allowance was active, but local turning-pocket
selection still rejected compound forward routes. A stationary capture at
approximately (-1.533, -0.621), heading -150.98 degrees, was probed toward
(1.23, -0.43) with explicitly hypothetical terminal yaw zero.

The updated `forward_route` replay keeps the robot at that original pose,
prepends the entire checked forward departure, and accepts the complete
SE2 routes through hypothetical forward starts 0.15, 0.45 and 0.60 m away.
The original start's SE2 route is still rejected at its unsafe intermediate
turn near (-2.633, -0.721); no collision cells or physical margins were removed.
Other tests cover length/turn ranking, trying the next route after geometric
execution failure, skipping a newly blocked choice, finite choice exhaustion,
planner cancellation/deadlines, fresh-data prefix checking past one metre,
wrong-goal rejection and clearing choices after success, failure or halt.
These tests never issue real velocity/navigation goals. Exact operator goal
orientation, dynamic obstacles and real tracking still need operator testing
after restart; success of every physically unreachable goal is not promised.

## Local oscillation regression, 2026-09-26

The live 16:29 launch logged two failed goals with repeated +/-15-degree
turns and 15 cm local advances, but no successful FollowPath execution during
those attempts. One goal aborted less than one second after reporting a
checked rear corridor. The subsequent goal did succeed; ordinary complete
route tracking is preserved by the region-memory change.

New isolated regressions cover two local adjustments surviving ordinary
replan/partial-turn resets, a single continuously running BackUp, release at
a different measured rear pose, immediate acceptance of a fully checked
goal-facing turn, a lost braking pocket, fault/rear-block rejection, one-shot
handoff consumption and new-goal memory reset. Pure helper tests cover signed
reverse displacement, sideways/forward rejection and an idempotent retreat
anchor using the actual reverse heading. Build succeeded and all three
selected CTest groups passed. The optional old capture replay was skipped
because its capture is not present. No real motion goal or launch restart
was performed for this revision; the operator must restart to load it.
