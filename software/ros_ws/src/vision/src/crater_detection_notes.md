# Crater detection: working notes

Running record of what has been established, decided and left open. Companion to
`crater_detection_agent_brief.md` (the current brief). Last updated 2026-10-13.

Status: **basic detector implemented on 2026-10-13 (section 11) and committed on
`feat/detect-crater`: the hole test is in the shared classifier, with unit tests, and both
nodes were run against a synthetic lidar and then in Gazebo on the cratered arena with the
rover driving (sections 11.7 to 11.9). Not yet run on real lidar data. This branch is 176
commits behind the branch the rest of autonomy is on (section 2.2). A summary for testers
is in `crater_detection_README.md`; a step-by-step
explanation of the code is in `crater_detection_walkthrough.md`.**

What changed in this revision, in one paragraph: the team reported that the rover gets
stuck or slowed in craters, and that some of those craters are ruts the rover digs itself
when it pivots. So the hazard set grows during a run and a one-off map of the arena cannot
be complete. The detector rule from the previous plan stands. What changes is everything
around it: detection has to work live on every pass, memory is split into a short-term part
that can be trusted and a long-term part that cannot, and self-made ruts get their own
handling because the lidar mostly cannot see them. Section 10 lists the differences from the
2026-10-08 plan.

## 1. Requirement

- 2026 Lunabotics guidebook, section 3.1: at least three craters of varying depth and
  width, no wider or deeper than 40-50 cm. Boulders are 30-40 cm.
- The guidebook figure shows craters in both the obstacle zone and the excavation zone;
  placement is random on the day.
- Deliverable is detect-and-avoid: crater cells marked lethal in the Nav2 costmaps.
  Crater size/centre fitting is optional and only matters for crater-based localisation.
- Constraints from the team: raw point cloud, no ML (labelling cost).
- **New, 2026-10-13:** this is now an observed failure, not only a rule. In a recent field
  test the rover "easily gets stuck or slowed" in craters (Kelvin). Some of those were made
  by the rover while turning (Shaan). The requirement therefore covers two kinds of hole:
  - **arena craters**, placed before the run, 40-50 cm class, static;
  - **self-made ruts**, dug by the wheels during pivots and by the bucket while digging,
    appearing during the run. Their size has not been measured (section 9).

## 2. What the repo already has

### 2.1 Verified by reading the code on this branch (`feat/detect-crater`)

| Item                    | Finding                                                                                                                                                                              | Where                                                                             |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------- |
| ROS / nav               | ROS 2 Jazzy, Nav2, target computer is an Orange Pi                                                                                                                                   | `nav-stack/README.md`, `navigation.yaml`                                          |
| Lidar                   | Livox MID-360 (3D), pitched 45 deg down on the mast                                                                                                                                  | `description/urdf/sensor_mount.urdf.xacro`, `lidar_angle` in `perseus.urdf.xacro` |
| Lidar height            | 0.733 m above `base_footprint` (measured from TF in sim, matches URDF arithmetic)                                                                                                    | sim capture                                                                       |
| Forward beam elevations | about -52 deg to +7 deg, so ground is only seen in a forward wedge                                                                                                                   | derived from the 45 deg pitch and the -7..+52 deg FOV                             |
| Camera                  | RealSense D455, looking straight ahead; depth and point cloud **disabled**, emitter off for stereo odometry                                                                          | `sensors/config/realsense.yaml`                                                   |
| Terrain pipeline        | `local_traversability` (8 m window, 10 Hz, forgets) and `global_traversability` (log-odds memory) share one classifier on a 0.1 m grid built from the last 1 s of raw `/livox/lidar` | `nav-stack/local_traversability/src/local_traversability/terrain_analysis.cpp`    |
| Negative-obstacle test  | exists: `max_step_down_m: 0.15`, with a slope allowance over a 0.5 m window                                                                                                          | same file, `compute_height_difference`                                            |
| Unobserved cells        | published as unknown and treated as free (three linked `UNMAPPED` settings)                                                                                                          | `navigation.yaml`                                                                 |
| RANSAC history          | a RANSAC ground fit was used on the accumulated LIO map and removed; the current test was chosen for being deterministic and cheap                                                   | `global_traversability.hpp` header comment                                        |
| Costmap frame           | every Nav2 `global_frame`, and both traversability grids, are in `odom`                                                                                                              | `navigation.yaml`                                                                 |
| Speed cap               | 0.28 m/s linear                                                                                                                                                                      | `velocity_smoother.max_velocity` in `navigation.yaml`                             |
| Recorded bags           | none in the repo; config comments reference a `rock_nav_test_2` bag                                                                                                                  | -                                                                                 |

Stale comments to fix when touching the code: `local_traversability.hpp` lines 37-47 and
121 still say `global_traversability` fits a plane by RANSAC. It no longer does. The
`navigation.yaml` header and `global_traversability.hpp` say "nothing publishes
map -> odom"; `arena_server` now does.

### 2.2 The branch the rest of autonomy is on

`origin/feat/autonomy-refactoring` (author bocho0600, taken to be Kelvin's work; last commit
2026-10-13) is 176 commits ahead of this branch and this branch has no commits of its own.
Everything the team described on 2026-10-13 lives there, not here. Read with `git show`, not
checked out or run:

| Item                   | On `origin/feat/autonomy-refactoring`                                                                                                                  | Matters for craters because                                                          |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------ |
| Terrain classifier     | `local_traversability` and `global_traversability` sources are **unchanged** from this branch                                                          | the detector design in section 5 still applies as written                            |
| Lidar input            | both nodes read `/livox/lidar/cropped` from a new `sensors/height_crop` node (drops points more than 1.5 m above the `odom` origin)                    | the ceiling cut now exists in a level frame, which makes the 7.5 bug simpler to fix  |
| Near range gate        | `min_range_m` raised from 1.0 to **1.6** (global) and **1.7** (local)                                                                                  | ground closer than about 1.4-1.5 m from the sensor is never classified (section 4.5) |
| Height gate bug (7.5)  | still present: `filter_scan` still drops `point.z > max_sensor_height_m` in the raw sensor frame (`terrain_analysis.cpp` line 357)                     | forward ground beyond about 2.85 m is still discarded, for rocks as well as craters  |
| Global confirm speed   | `log_odds_hit: 0.25` against `log_odds_occupied: 0.5`, `update_period_s: 0.5`; the config says a small obstacle is confirmed after about 1.5 s in view | too slow for a crater that is only visible for about 0.5 m of travel (section 4.5)   |
| Inflation              | `robot_radius_m: 0.6` (half-width 0.435 m plus about 0.17 m), `inflation_radius_m: 0.8`, `cost_scaling_factor: 20.0`                                   | the 0.17 m margin is what absorbs a small error in a remembered crater's position    |
| Planners               | `ThetaStar` (still the default), `GridBased`, and a new heading-aware `Lattice` (Smac State Lattice, 1 m arcs, real footprint)                         | section 3, row 5                                                                     |
| Pivot thresholds       | RPP `rotate_to_heading_min_angle` 1.0 rad; DWB behind a rotation shim with `angular_dist_threshold` 2.4 rad                                            | section 3, row 5                                                                     |
| Second odometry source | ORB-SLAM3 stereo (`vision/orb_slam_odometry`) replaces libviso2 as the EKF's `odom1`                                                                   | section 3, rows 6 and 10                                                             |
| Bucket poses           | travel pose lift 25 deg, tilt -20 deg (about 0.17 m ground clearance); **carry pose after a dig is lift 0, tilt -28**                                  | lift 0 is the pose section 7.4 found blind dead ahead (section 4.6)                  |
| Mission cycle          | excavation -> dig -> construction -> dump -> excavation, repeated; waypoints from `arena_server`, scored against `/costmap`                            | the same legs are driven every cycle, and the rover turns round at each end          |
| Slip detection         | `mobility_watchdog` publishes `/watchdog/mobility_status`; only RViz panels read it                                                                    | an unused signal that says "the rover is bogging here" (section 5.5)                 |

**Consequence: crater work should be based on `origin/feat/autonomy-refactoring`, not on
this branch.** `navigation.yaml` alone differs by 245 lines. Since this branch carries no
commits, rebasing costs nothing now and grows with every day of delay.

## 3. Team input on 2026-10-13, checked against the code

Relayed from a team discussion. "Code" below means `origin/feat/autonomy-refactoring` unless
stated.

| #   | What was said                                                                                                                                       | What the code shows                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Consequence for craters                                                                                                             |
| --- | --------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| 1   | Kelvin: the rover easily gets stuck or slowed in craters. Kelvin can take rock-detection testing; crater detect is needed.                          | Not checkable from the repo. Consistent with 4.2: the existing `step_down` test misses most craters, so nothing stops the planner routing through one.                                                                                                                                                                                                                                                                                                               | Crater avoidance is an observed failure. Ownership: craters here, rocks with Kelvin.                                                |
| 2   | Shaan: some of that was Perseus making craters when it turned.                                                                                      | The branch's own config comments agree: "pivoting a skid-steer in regolith is the costly manoeuvre (it digs in, slips, and sits near the ESC stall)". Wheels sit 0.43 m (front) and 0.72 m (rear) from `base_link`, so one pivot's ruts lie inside a circle of about 0.75 m radius.                                                                                                                                                                                  | Hazards are created during the run, under the rover, where the lidar cannot see. See 4.7.                                           |
| 3   | Shaan: scan for craters on the first trip from the start zone to the construction zone and keep that as the reference for craters we did not cause. | `global_traversability` already remembers every cell it classifies for the whole run, so that survey happens without a separate step. Nothing records when a cell was first seen, so "not caused by us" is not available today.                                                                                                                                                                                                                                      | Section 5.6: adopted as a cheap tag, not as something avoidance depends on.                                                         |
| 4   | Shaan: does not trust a map on the way back, because of drift in the lidar odometry.                                                                | The costmap is in `odom`. Nothing corrects `odom` (rows 8 and 10). The size of the drift over a cycle has not been measured anywhere in the repo. The one number available is vertical: LIO drifted 0.6 m in z on `rock_nav_test_2` (`ekf_config.yaml`).                                                                                                                                                                                                             | The distrust is justified by the code as it stands. Section 5.4 is built around it.                                                 |
| 5   | Kelvin: turning and path planning now avoid pivoting at one point as much as possible.                                                              | True in three places: the `Lattice` planner, RPP's pivot threshold at 1.0 rad ("only pivot when the path is nearly behind (sand: avoid pivoting)"), and DWB's shim pivoting only when the path is about 140 deg off. Two limits: `ThetaStar` is still the default planner ("until Lattice has been compared against it on the zone cycle"), and Lattice's weak case is turning round for a goal (526 deg of pivoting in 9 pivots, against ThetaStar's 321 deg in 2). | Fewer pivots, not zero. The turn-round at each zone waypoint remains, at the same places every cycle.                               |
| 6   | Kelvin: LIO is known to drift, which is why the EKF fuses ORB-SLAM with it.                                                                         | True on the branch. ORB-SLAM3 is `odom1`, fused **differentially**: only the change between consecutive poses is used. On replay its 5 s displacement was 0.94x the LIO's. On this branch `odom1` is still libviso2.                                                                                                                                                                                                                                                 | A second dead-reckoning source reduces drift rate. It cannot remove drift: neither source has an absolute reference.                |
| 7   | Kelvin: BIEVR-LIO now estimates covariance per scan from the Gauss-Newton Hessian, so `/Odometry` carries a meaningful R.                           | True on both branches (`nix/extra-packages/patches/bievr-lio/src.nix`, `poseCovarianceDiagonal`). A floor of 0.0004 m^2 is applied because the raw Hessian is overconfident (about 1e-5 m^2 against about 15 mm RMS scan-to-scan jitter).                                                                                                                                                                                                                            | Better blending and better behaviour in degenerate geometry. Same limit as row 6.                                                   |
| 8   | Kelvin: if odometry drifts or diverges, ArUco tag odometry is the ground truth.                                                                     | `arena_server` offers `/arena/localise`, a **service**: on demand, needs at least two rail markers that agree within 0.25 m and 5 deg, and it updates `map -> odom` only. Nothing in the behaviour trees, `mission_bt_server` or the RViz plugins calls it; the only mention there is a panel comment. Single-marker yaw error of up to 9.4 deg is recorded in the config.                                                                                           | A fix corrects where goals land. It does not move remembered costmap cells, which stay in `odom`, and it does not happen by itself. |
| 9   | Kelvin: the point cloud map is only for visualisation.                                                                                              | True. `global_traversability` reads raw scans; `/Laser_map` goes to the base-station downlink only.                                                                                                                                                                                                                                                                                                                                                                  | No change. The crater detector also works from raw scans.                                                                           |
| 10  | Kelvin: two SLAMs run at once, BIEVR-LIO and ORB-SLAM, and ORB-SLAM should have loop closing.                                                       | **Not as configured.** `vision.yaml` sets `loop_closing: false`, and the README says "Loop closing is off". The node also publishes only frame-to-frame steps and replaces any pose jump with an identity step at variance 9999, so a loop-closure correction could not reach the EKF even if it were switched on.                                                                                                                                                   | There is no loop closure anywhere in the pose chain. Worth raising with Kelvin, since row 4 depends on it.                          |

Summary of the localisation picture, because the plan leans on it: `odom` is two fused
dead-reckoning sources with no absolute correction. `map -> odom` can be corrected by ArUco,
on request. The costmap, and so any remembered crater, lives in `odom`. Over seconds this is
as good as the lidar itself. Over a full cycle it is unmeasured.

## 4. Findings so far

### 4.1 RANSAC is not the right detector

- A single-plane fit is wrong for terrain with berms, mounds and the rover's own digging.
- Per-patch fits cost too much for 10 Hz on the Orange Pi and are non-deterministic.
- The useful part of the idea ("how far below the expected ground is this?") can be had
  with a deterministic regional median.
- Legitimate remaining use: fitting a circle to an already-detected crater blob, only if
  crater-based localisation is wanted.

### 4.2 The existing `step_down` test will miss most craters (simulation estimate)

Idealised 2D slice through a crater centre, lidar at 0.73 m, thresholds from
`navigation.yaml`. No noise, no real data.

| Crater (width x depth) | Distance | Cells flagged by `step_down` | Interior cells in shadow | Deepest return below ground |
| ---------------------- | -------- | ---------------------------- | ------------------------ | --------------------------- |
| 0.5 x 0.3 m bowl       | 1.5 m    | 0                            | 3 of 4                   | 0.22 m                      |
| 0.5 x 0.3 m bowl       | 2.5 m    | 0                            | 3 of 4                   | 0.14 m                      |
| 1.0 x 0.3 m bowl       | 2.5 m    | 0                            | 5 of 10                  | 0.25 m                      |
| 1.0 x 0.3 m steep      | 2.5 m    | 1                            | 7 of 10                  | 0.30 m                      |
| 1.0 x 0.5 m steep      | 1.5 m    | 4                            | 5 of 10                  | 0.50 m                      |

Two causes: the slope allowance absorbs the drop, and most of the interior is shadow, which
the stack treats as free. The usable signal is the far-wall return, which sits well below
ground level.

### 4.3 Geometry of the signal

The beam that clears the near rim lands on the far wall roughly
`width x 0.73 / distance` below the rim (capped by the crater depth):

| Hole width           | At 1.6 m | At 2 m | At 3 m | At 4 m |
| -------------------- | -------- | ------ | ------ | ------ |
| 0.15 m (a wheel rut) | 0.07 m   | 0.05 m | 0.04 m | 0.03 m |
| 0.4 m                | 0.18 m   | 0.15 m | 0.10 m | 0.07 m |
| 0.5 m                | 0.23 m   | 0.18 m | 0.12 m | 0.09 m |

The rut width of 0.15 m is an assumption; the wheel width has not been looked up.

### 4.4 Avoidance does not need to know who made the hole (new)

A hole deep enough to bog the rover is a hazard whether the arena staff or the rover dug it.
The detector is geometric and runs on live scans, so it flags both the same way. Telling them
apart is only needed for:

- crater-based localisation (a self-made rut is not a landmark), which is not planned;
- measuring how much the pivot-avoidance work helps (a count of new holes per cycle);
- deciding how hard to avoid a spot (section 5.5).

So "which craters did we not cause" is a diagnostic, not a prerequisite. This is what lets
the plan avoid depending on a reference map.

### 4.5 Memory is needed at two timescales, and only one of them is at risk from drift (new)

**Seconds: bridging the near blind zone.** With `min_range_m` at 1.6 and 1.7 and the lidar
0.733 m up, ground closer than about 1.42 m (global) or 1.53 m (local), measured
horizontally from the sensor, is never classified. `local_traversability` forgets within
1 s. So for the last metre and a half before the rover reaches a crater, the only thing
holding it in the costmap is `global_traversability`'s memory. That memory is seconds old,
the rover has moved a metre or two, and drift over that span is negligible. **This memory is
required and can be trusted.**

It also has to be fast. The shallow sim crater (0.40 x 0.15 m) is detected out to 2.0 m
(7.2), so it is visible from 2.0 m down to about 1.5 m: 0.5 m of travel, about 1.8 s at
0.28 m/s. The global node currently needs about 1.5 s of sightings to confirm a cell. That is
too close to call, so hole verdicts need to be confirmed in one update (5.3).

**Minutes: the return leg and later cycles.** A crater seen on the way out is still in the
grid when the rover comes back, at the `odom` coordinates it had then. If `odom` has drifted
in between, the remembered cells are offset from the real hole. This is Shaan's concern and
the code gives no reason to dismiss it (section 3, rows 4, 8 and 10). **This memory is
useful for route choice and must not be the thing that keeps the rover out of a hole.**

What limits the damage: remembered cells are re-decided whenever the lidar looks at them, so
on approach a misplaced crater is cleared and the real one is marked, both from live data. An
error smaller than the 0.17 m inflation margin is absorbed without any of that.

### 4.6 The loaded leg is driven in the pose that blinds the lidar (new)

`DigBucket` ends in the carry pose, lift 0 and tilt -28, and the rover drives from
excavation to construction like that. Lift 0 is the pose in which section 7.4 measured
44-59% of ground cells seen within 10 deg of the nose and missed the crater dead ahead. The
travel pose (lift 25, tilt -20) is only restored after the dump.

- The 7.4 sweep did not test lift 0 with tilt -28, or lift 25 with tilt -20. Its nearest
  rows are lift 0 (blind) and lift 20-30 (64-96% seen). Both mission poses need measuring.
- If the carry pose is as blind as lift 0 was, no detector helps on that leg: the data is
  not there. That leg would rely entirely on long-term memory, the part section 4.5 says
  not to rely on.
- This is a mission-design question (can the load be carried lower?), not a detector one.

### 4.7 The lidar will not see most self-made ruts (new)

- They are made under the rover, inside the near blind zone and partly behind it. They can
  only be seen after the rover has driven away and looks back, or on the next approach.
- From the 4.3 table, a rut one wheel wide gives a far-wall signal of 0.05-0.07 m at
  1.6-2 m. That is below the 0.08 m threshold, and section 7.4 showed that lowering the
  threshold to 0.06 starts flagging natural dips.
- A full turn on the spot may instead leave a broad shallow bowl about 1.5 m across. Section
  7.4 found that a broad dip about 8 cm deep is correctly left alone, so a bowl shallower
  than that is invisible by design.
- Neither shape has been measured. What bogged the rover in the field test, and how deep it
  was, is the most useful missing fact (section 9).

Conclusion: prevention (Kelvin's pivot avoidance) and knowledge of where the rover itself
turned or slipped are better tools for ruts than the lidar is.

## 5. The plan

### 5.1 Principles

1. **Avoid by geometry, not by origin** (4.4). One detector, all holes.
2. **Detect live on every pass.** The live detection in the current scan is what keeps the
   rover out of a hole. Nothing relies on a map built earlier in the run.
3. **Trust memory over seconds, doubt it over minutes** (4.5). Short-term memory bridges the
   near blind zone. Long-term memory informs the planner and corrects itself on approach.
4. **Ruts need non-lidar handling** (4.7): prevention, plus marking where the rover pivoted
   or slipped.
5. One classifier, shared by both traversability nodes. No new Nav2 layer, no RANSAC, no ML.

### 5.2 Phase 0: unblock (no detector code)

Small, and each item changes what the later phases look like.

1. **Move to `origin/feat/autonomy-refactoring`** (2.2). Agree with Kelvin whether to branch
   from it or wait for it to reach `main`.
2. **Get the field-test facts** (section 9): which holes bogged the rover, their width and
   depth, and where in the cycle it happened.
3. **Fix the height gate (7.5).** With `height_crop` now cutting the ceiling in a level
   frame, the sensor-frame gate in `filter_scan` is redundant for that job. Confirm on the
   real rover that `/local_layers/elevation` is empty beyond about 3 m ahead, then remove or
   re-frame the gate. This also extends rock detection, so it concerns Kelvin's testing now.
4. **Measure the two mission bucket poses in sim** (4.6): travel (25, -20) and carry
   (0, -28), with the same visibility sweep as 7.4.
5. **Measure odometry drift over one cycle** (5.4, 8).

### 5.3 Phase 1: the live hole detector

One new test inside the existing classifier, lidar only. Unchanged from the previous plan
except where marked.

**Rule: a cell is a hole when lidar rays are observed travelling below where the ground
should be.** This is positive evidence from real returns, so sparse scans cannot produce
false holes, and one rule covers both the low far-wall return and the shadowed interior.

1. **Ground reference.** Per cell, from the same 1 s scan buffer, using only cells the
   existing step-up and height tests do not flag (see 7.2): median elevation over
   0.5 m blocks, 3x3 median filter across blocks, interpolated back to 0.1 m.
2. **Sub-ground rays.** For each return more than `hole_depth_m` below the reference,
   walk back along the ray toward the sensor and mark each cell crossed while the ray is
   still below the reference.
3. **Verdict.** A cell is a hole when at least `hole_min_rays` rays marked it. OR it into
   the existing `obstacle` layer, so pruning, inflation, costmap publishing and Nav2 are
   unchanged. Also keep it as its own `hole` layer, because phase 2 treats hole cells
   differently from rocks.
4. **Fast confirm (new).** In `global_traversability` a hole verdict adds
   `hole_log_odds_hit`, set above `log_odds_occupied` so that one update marks the cell
   (4.5). The slow `log_odds_hit: 0.25` exists to reject flickering single returns; a hole
   verdict has already passed a `hole_min_rays` test.

Starting parameters, to be tuned on data:

| Parameter           | Start | Basis                                                                            |
| ------------------- | ----- | -------------------------------------------------------------------------------- |
| `hole_depth_m`      | 0.08  | 0.06 flags a natural dip; 0.07 and 0.08 give zero false cells (7.4)              |
| `hole_min_rays`     | 3     | untested alternative values                                                      |
| `hole_max_range_m`  | 4.0   | ground noise reaches 0.09 m at 4-5 m (7.2)                                       |
| `hole_log_odds_hit` | 0.7   | one update crosses `log_odds_occupied` 0.5; matches the header default for rocks |

Expected performance, from the sim prototype: 40-50 cm craters detected from 2-3 m and
tracked until they pass inside the near range gate at about 1.5 m. At 0.28 m/s that is 2-5 s
of warning. These ranges were measured with the old 1.0 m near gate and the height-gate bug
present, and with the rover stationary; they need re-measuring on the new base.

Files that change:

- `terrain_analysis.hpp/.cpp`: sensor origin stored with each buffered scan, the two new
  passes, the parameters.
- `global_traversability.cpp`: the hole hit increment and the phase 2 rules.
- `navigation.yaml`: parameters and the measurements behind them.
- Debug layers `ground_ref`, `hole_depth`, `hole` under `/local_layers/`.

RealSense: not used. Depth is disabled and the emitter is off because ORB-SLAM3 uses the
infra pair. The test only needs a sensor origin and points, so a depth cloud could feed the
same code later.

### 5.4 Phase 2: memory that survives drift

**Short-term (required).** Falls out of phase 1: the fast confirm puts the hole in the global
grid while it is visible, and unobserved cells keep their value, so the hole stays put while
it is inside the near blind zone. Nothing else to build. Test it directly (section 8, test 4).

**Long-term (aid).** Two rules in `global_traversability`:

1. **Clear only from close range.** A hole cell loses log-odds only when it is observed as
   flat from within about 2.5 m. Otherwise a crater seen clearly at 2 m is erased when
   viewed from 5 m, where it reads as flat. This was in the previous plan.
2. **Let live data win.** No special handling is needed for a remembered hole that has
   drifted: when the rover comes back, the real hole is marked from live rays and the stale
   cells are cleared by rule 1 once the lidar sees flat ground there from close range (8
   updates, about 4 s, from the log-odds ceiling).

**Decision gate: measure drift before building anything more.** Drive one full cycle and
compare the pose at the same physical spot at the start and the end (section 8, test 6).

| Drift in x/y over one cycle | Action                                                                                                           |
| --------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| under about 0.15 m          | inside the inflation margin; the two rules above are enough                                                      |
| 0.15 m to about 0.5 m       | age long-term hole cells: after `hole_memory_s` an unrefreshed hole drops from lethal to a high non-lethal cost  |
| over about 0.5 m            | a localisation problem, not a crater one; take it to Kelvin (ArUco fix at each zone arrival, grid kept in `map`) |

The thresholds are proposals. 0.15 m comes from the 0.17 m inflation margin; 0.5 m is a
crater width.

Why not simply keep the grid in `map` and correct it from ArUco now: every Nav2
`global_frame` is `odom`, a fix moves the whole grid at once, and a bad single fix has been
measured at 9.4 deg of yaw. That is a larger and riskier change than the drift it would cure
has been shown to justify.

### 5.5 Phase 3: self-made ruts

Proposals, since they touch the planner and the mission tree, which are Kelvin's. In order
of value for effort:

1. **Prevention is first, and already under way** (section 3, row 5). Open follow-ups that
   would reduce ruts further: making `Lattice` the default once compared, and whether the
   turn-round at each zone can be an arc or a reverse rather than a pivot.
2. **Mark pivot sites as soft cost.** When the rover rotates more than a set angle with
   almost no forward motion, stamp a disc of about 0.75 m radius around `base_link` (the
   reach of the rear wheels) with a high, **non-lethal** cost. The planner then prefers
   fresh ground but can still cross if there is no other way. Non-lethal on purpose: the
   position is only as good as long-term odometry (4.5), and the arena is small.
3. **Mark slip sites the same way.** `/watchdog/mobility_status` already reports sustained
   slip and nothing acts on it. A slip report is direct evidence of ground that bogs the
   rover, whatever made it and whether or not the lidar can see it.
4. **Lidar, where it can.** Phase 1 will catch a rut that is wide and deep enough. Do not
   lower `hole_depth_m` to chase narrow ruts; 7.4 shows what that costs.

One interaction to check before doing 2 or 3: `arena_server` rejects zone waypoints above
`waypoint_max_cost` 50. Soft cost above that would make it pick a fresh spot each cycle,
which spreads the wear but could exhaust the 2.05 x 3.41 m construction zone. Soft cost
below 50 leaves waypoint choice alone.

### 5.6 Shaan's baseline survey: what is adopted

Not adopted as designed: a reference map from the first traverse that later avoidance depends
on. It covers only the corridor the lidar swept on that leg, it is exactly the long-lived map
4.5 says not to depend on, and avoidance does not need it (4.4).

Adopted as a cheap tag, if phase 1 and 2 are done and time allows: record for each hole cell
when it was first seen, and whether the rover's footprint had already passed over that cell.
A hole that appears where the rover has already driven is self-made by definition. That gives
the team a count of new holes per cycle, which is the measure of whether pivot avoidance is
working, and a filter for landmarks if crater-based localisation is ever wanted. It is a
diagnostic and its accuracy is limited by the same drift.

### 5.7 Known limits

- The lidar only sees ground in a forward wedge beyond about 1.5 m; holes beside or behind
  the rover rely on memory.
- Holes shallower than about 8-10 cm, and narrow ruts, will be missed by the lidar.
- If the carry pose blinds the lidar (4.6), the loaded leg has no live detection.
- The rover's own excavation pit will be flagged as a hole. That is correct behaviour.
- Reversing is unprotected: DWB may reverse at up to 0.15 m/s and there is no rear ground
  view.

## 6. Simulation environments

Sim repo: `~/perseus_simulation` (pixi, ROS 2 Jazzy, Gazebo Sim 8). The sim runs on
`ROS_DOMAIN_ID=51` with `rmw_cyclonedds_cpp`; a shell needs both set to see its topics.
Run tools through `scripts/pixi-run.sh` so system ROS does not leak in.

|                 | `feature/lunabotics-arena`                                          | `main` (ARC world)                                                         |
| --------------- | ------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Terrain         | 9.14 x 8.10 m UCF arena, walls, zones, fiducial rail                | 26 x 30.5 m dune                                                           |
| Heightmap       | 129 x 129 PNG, about 7 cm per cell, 0.30 m relief, slopes to 18 deg | 33 x 33 GeoTIFF, about 0.8 x 0.95 m per cell, 1 m relief, slopes to 15 deg |
| Craters         | **none modelled** (stated in the `Moon_DEM/model.sdf` comment)      | broad dips only: deepest is 0.12 m below its 1.5 m neighbourhood           |
| Boulders        | seven at 30-40 cm, guidebook scale                                  | Falling Rock models at default (metre) scale                               |
| Simulated lidar | 720 x 96, mount matches perseus-v3 (0.733 m, 45 deg)                | 720 x 32, different mount definition (`livox_pitch`)                       |
| Other           | `RegolithDig` plugin carves the bed at run time                     | ArUco cubes, `/gazebo_odom` ground truth                                   |

Neither world has a `odom` TF frame or the nav stack running by default; scans were
captured in `base_footprint` with the rover stationary.

**Decision: use `feature/lunabotics-arena`, with three guidebook craters carved into its
heightmap.** Reasons:

- The ARC world's raster cells are larger than a whole crater. It cannot represent a
  40-50 cm hole, and its dips are gentle undulations that a crater detector should
  correctly ignore.
- The arena branch is at guidebook scale, has the right boulders, and its lidar mount
  matches the real rover.
- Its heightmap is fine enough to carve craters into (about six cells across a 45 cm
  crater).

Carving needs care: craters deeper than the current 0.30 m relief require a larger
heightmap Z size, re-normalising the PNG, and updating **both** Z offsets (model `<pose>`
for collision, visual `<pos>`), as the model's own comment explains.

The ARC world stays useful as a false-positive check on undulating ground.

## 7. Sim work so far (2026-10-08, on the pre-refactor config)

### 7.1 Craters carved into the arena (loaded and checked in 7.4)

Branch `feat/arena-craters` in `~/perseus_simulation`, off `feature/lunabotics-arena`,
uncommitted:

- `src/perseus_simulation/scripts/carve_craters.py` writes
  `models/Moon_DEM/dem/regolith_heightmap_craters.png` (16-bit, absolute,
  `z = -0.512 + pixel / 65535 * 1.0`, the encoding `RegolithDig` itself writes).
- `Moon_DEM/model.sdf` and the `RegolithDig` block in the world point at it, with size z
  1.0 and offset -0.512. All three places must agree.

| Crater | Position (x, y) | Width  | Depth  | Profile | Note                                     |
| ------ | --------------- | ------ | ------ | ------- | ---------------------------------------- |
| 1      | (-1.20, 3.80)   | 0.50 m | 0.45 m | steep   | dead ahead of the spawn pose, 2.7 m out  |
| 2      | (0.60, 5.30)    | 0.45 m | 0.30 m | bowl    |                                          |
| 3      | (-2.20, 6.00)   | 0.40 m | 0.15 m | bowl    | shallow end of the range                 |
| 4      | (2.00, 6.30)    | 0.50 m | 0.45 m | steep   | 0.50 m would go below the bed's floor    |
| 5      | (1.20, 2.00)    | 0.45 m | 0.40 m | bowl    | excavation zone, abeam of the spawn pose |

The raster is about 7 cm per vertex, so a crater is 26-41 vertices and its walls are
faceted. The sim must be restarted (`pixi run sim`) to load it; whether Gazebo renders
and collides it correctly is **not yet checked**.

### 7.2 Offline prototype on captured scans

Ten scans (1 s) captured at the spawn pose of the crater-free arena, rover stationary.
Craters were injected by re-casting every ray that landed inside a crater disc onto the
crater surface. Scratch code, not in either repo.

Results:

- **False positives:** zero hole cells at `hole_depth_m` 0.06, 0.08 and 0.10 on the bumpy
  bed with boulders and walls in view; 18 cells at 0.05. One stationary pose only.
- **Ground noise floor:** ground cells sit within about 6 cm of the reference inside 3 m
  (99th percentile 0.02-0.06 m), rising to 0.09 m at 4-5 m.
- **Detection at clear bearings (30 and 45 deg off the nose):**

| Crater (width x depth) | Threshold 0.08 m             | Threshold 0.06 m |
| ---------------------- | ---------------------------- | ---------------- |
| 0.40 x 0.15 m bowl     | out to 2.0 m                 | out to 2.5 m     |
| 0.45 x 0.30 m bowl     | 2.0 m (2.5 m at one bearing) | 3.0 m            |
| 0.50 x 0.45 m steep    | 3.0 m                        | 3.5 m            |

- **Geometry estimate confirmed:** deepest return below the reference was 0.12-0.13 m at
  2.5 m and 0.10-0.12 m at 3 m for the 0.5 m crater, matching section 4.3.
- **The existing `step_down` test** flagged at most a handful of cells, and only at 1.5 m.

Two corrections to the plan came out of this:

1. **The ground reference must be built from ground-like cells only.** With every cell
   included, wall and building returns pulled the block medians up by metres and produced
   798 false hole cells. Excluding cells the existing step-up and height tests already
   flag, and applying the node's own sensor-frame gates, brought that to zero.
2. **0.06 m looks like a better starting threshold than 0.08 m** in sim: same zero false
   positives, about 0.5-1 m more range. Sim has no sensor noise, so this needs real data.

Caveat: the simulated lidar is 96 fixed rings, so at 3.5 m only two or three rings enter
a crater. The real MID-360's non-repetitive pattern should be denser over 1 s.

### 7.3 The bucket blocks the lidar dead ahead (new, important)

In the sim, with the bucket in its start-up pose, every forward beam between about -50 deg
and -5 deg elevation hits the rover itself at 0.27-0.8 m range (self-hits span x 0.20-0.84,
y +/-0.42, z up to 0.70 in `base_footprint`).

| Bearing off the nose | Cells observed at 2-2.5 m | at 2.5-3 m | at 3-4 m |
| -------------------- | ------------------------- | ---------- | -------- |
| 0 deg                | 10%                       | 9%         | 0%       |
| +/-10 deg            | 5-11%                     | 16-20%     | 0%       |
| +/-20 deg            | 81-90%                    | 55-68%     | 2%       |
| +/-40 deg            | 100%                      | 96-100%    | 60-64%   |

Consequence: injected craters dead ahead were found at 1.5-2.0 m and never at 2.5 m or
beyond, for every size. No algorithm fixes this; the data is not there.

Team intent (2026-10-08): travel with the bucket posed to obstruct the lidar as little as
possible. Section 7.4 measures which poses do that in sim.

### 7.4 Tests against the carved craters (sim restarted with the cratered bed)

Rover stationary at the spawn pose, 1 s of scans per test, same offline prototype.

**Craters load and are visible to the lidar.** Crater 5 (0.45 x 0.40 m, 2.5 m away, 69 deg
off the nose) was detected in every run, with 10-12 hole cells and a deepest return
0.14 m below the reference. The existing `step_down` test flagged zero cells on it.

**Bucket pose sweep** (lift 0 deg = arms level at the top of travel, positive lowers;
tilt negative = curled):

| Lift                  | Self-hits per scan | Ground cells seen within 10 deg of the nose, 1.5-2.5 m | Crater 1 (dead ahead, 2.7 m) |
| --------------------- | ------------------ | ------------------------------------------------------ | ---------------------------- |
| 0 deg (start-up pose) | about 2400-2800    | 44-59%                                                 | missed, 2 of 39 cells seen   |
| 10 deg                | about 1700-1900    | 30-53%                                                 | not tested                   |
| 20 deg                | about 1150-1500    | 64-90% (best with tilt 0)                              | detected, 13 hole cells      |
| 30 deg and lower      | about 750-1100     | 96%, any tilt                                          | detected, 13-19 hole cells   |

At lift 30 deg the forward beams reach the ground from about 1.3 m outward; closer than
that the ground is under the bucket. No regolith was cut down to lift 40 deg (bucket
pivot 0.07 m above the bed). The sim bucket was left at lift 40 deg, tilt -90 deg.

**Threshold, bucket lowered:**

| `hole_depth_m` | Crater 1 hole cells | Crater 5 hole cells | False hole cells           |
| -------------- | ------------------- | ------------------- | -------------------------- |
| 0.06           | 19                  | 12                  | 31 (one blob, 1.7 m ahead) |
| 0.07           | 18                  | 10                  | 0                          |
| 0.08           | 15                  | 10                  | 0                          |
| 0.10           | 11                  | 6                   | 0                          |

The false blob at 0.06 is a real, broad natural dip in the bed, about 8 cm below the
regional reference. So 0.06 is too tight for this bed: **back to 0.08 as the starting
value**, which reverses the suggestion in 7.2. A broad shallow dip and a far crater give
similar depth evidence, so a threshold alone will not separate them at long range; a
width or shadow cue may be needed and is not designed yet.

Craters 2, 3 and 4 are 4.5-6 m from the spawn pose and were not detected from there, as
expected. Nothing has been tested with the rover moving.

### 7.5 Existing bug: the height gate discards forward ground beyond about 2.85 m

`filter_scan` in `terrain_analysis.cpp` drops points with `point.z > max_sensor_height_m`
(1.5 m) in the **raw sensor frame**. The lidar is pitched 45 deg down and the driver's
extrinsic is all zeros (`sensors/config/livox_config.json`), so sensor z is not "up": a
ground point straight ahead at distance d has sensor z of about 0.707 x (d - 0.73), which
exceeds 1.5 m once d passes about 2.85 m.

- Seen in the sim data: with that gate, no points remain ahead of the rover beyond 3.0 m;
  with the gate applied in a level frame, ground cells are observed out to 5 m.
- It affects both traversability nodes and everything they detect, rocks included. The
  global node's `max_range_m: 10.0` cannot take effect straight ahead.
- Earlier prototype numbers (7.2) used this gate, so their ranges beyond about 3 m forward
  are pessimistic.
- This assumes the real lidar is mounted with the tilt the URDF states.
- Likely fix: apply the height cut after rotating into a gravity-aligned frame (rotation
  only, still relative to the sensor, so still immune to z drift). Not implemented.

### 7.6 What 7.1-7.5 do not cover

All of section 7 was done on the configuration of this branch with the rover stationary. Not
yet covered:

- the rover moving, and so any test of memory;
- the new near range gates of 1.6 and 1.7 m (the prototype used the gates of the time);
- the mission's actual bucket poses, travel (25, -20) and carry (0, -28) (4.6);
- any self-made rut. The sim's `RegolithDig` plugin carves the bed where the bucket cuts;
  whether wheels deform the bed has not been checked, and nothing here assumes they do.

## 8. Validation plan

1. Unit tests on synthetic clouds: flat, 20 deg slope, rock, craters of 40-50 cm at 1.5,
   2.5 and 4 m, and a 0.15 m wide rut (expected: not detected, recorded as a known limit).
2. Sim, lunabotics arena without craters: false-positive count must be zero on the bumpy
   bed and around boulders.
3. Sim, lunabotics arena with carved craters, **rover driving**: approach each from several
   angles at 0.28 m/s; target is detection by 2 m every time.
4. **Short-term memory:** drive straight at a crater and keep going until the planner
   diverts. The hole must stay lethal in `/costmap` from first detection until the rover has
   passed it, including while it is inside the near range gate.
5. **Bucket poses:** repeat the 7.4 visibility sweep for travel (25, -20) and carry
   (0, -28).
6. **Drift over a cycle:** on the real rover, call `/arena/localise` in the start zone,
   drive one full excavation-construction-excavation cycle, return to the same spot and
   call it again. The change in `map -> odom` is the accumulated `odom` drift. Gate each fix
   on `residual_m`. In sim, compare against ground-truth pose if the arena world publishes
   one (`/gazebo_odom` exists in the ARC world; not checked for the arena branch).
7. **Long-term memory:** in sim, see a crater, drive away for a few minutes, inject a known
   offset into `odom`, return. The stale cells must clear and the real crater must be marked
   before the rover reaches it.
8. Replay `rock_nav_test_2` and count lethal cells under the driven path, the method the
   existing thresholds were tuned with. Target zero.
9. Real hole in sand: a 45 x 45 cm crater and a shallow 40 x 15 cm one, approached from
   5 m at several angles.
10. **Real rut:** pivot 360 deg on the spot in sand, measure the rut's width and depth,
    drive away and approach it from 5 m. Record whether the detector fires. This decides how
    much of phase 3 is needed.

Sim caveats: the simulated lidar is a fixed-ring scanner with little or no noise, while
the real MID-360 has a non-repetitive pattern. Sim results bound the geometry, not the
noise; the threshold still has to be confirmed on real data.

## 9. Open questions

For the team, most useful first:

- **What bogged the rover in the field test?** Arena-style craters, pivot ruts, or the dig
  pit? How wide and deep? Where in the cycle? Any bag or photos? This sets the balance
  between phase 1 and phase 3.
- **Kelvin: is loop closing meant to be on?** It is off in `vision.yaml`, and the odometry
  node is built to discard the pose jumps loop closure produces (section 3, row 10). If the
  intent was a drift-corrected pose, that is not what the stack does today.
- **Kelvin: is `/arena/localise` meant to be called during a mission?** Nothing calls it now
  (section 3, row 8).
- **Kelvin: branch from `feat/autonomy-refactoring` now, or wait for it to merge?**
- **How much does `odom` drift in x/y over one cycle?** Unmeasured (test 6).
- **Can the load be carried lower than lift 0?** (4.6)
- **Height gate (7.5):** confirm on the real rover, then fix. Affects rocks too.
- **Are soft-cost pivot and slip marks acceptable to the planner owner?** (5.5)

Carried over:

- Dips versus craters: how to reject broad shallow dips without losing range (7.4).
- Are holes under about 10 cm deep acceptable to drive through (wheel radius 0.15 m)?
- Where is the `rock_nav_test_2` bag stored?

Resolved since 2026-10-08:

- _Bucket travel pose._ The team chose lift 25, tilt -20, with about 0.17 m of ground
  clearance. Its effect on the lidar is still to be measured (test 5).
- _Should the excavation zone be masked?_ No. The dig pit is one more self-made hole and is
  handled like the rest (4.4).
- _Is crater-based localisation wanted?_ Not planned. No circle fitting.

## 10. What changed from the 2026-10-08 plan

| Topic           | 2026-10-08                                | 2026-10-13                                                                             |
| --------------- | ----------------------------------------- | -------------------------------------------------------------------------------------- |
| Why             | guidebook requirement                     | observed failure in a field test                                                       |
| Hazard set      | static arena craters                      | arena craters plus ruts made during the run                                            |
| Detector rule   | sub-ground rays against a regional median | unchanged                                                                              |
| Global confirm  | existing log-odds increment               | hole verdicts confirm in one update                                                    |
| Memory          | one rule: decay only from close range     | short-term (trusted, required) and long-term (aid, self-correcting), with a drift gate |
| Self-made holes | noted as a known limit (the dig pit)      | own phase: prevention, pivot and slip marks, optional first-seen tag                   |
| Base branch     | this branch                               | `origin/feat/autonomy-refactoring`                                                     |
| Near range      | 1.0 m                                     | 1.6 / 1.7 m, which makes short-term memory load-bearing                                |
| Bucket          | "travel with the bucket low"              | travel pose settled; carry pose identified as the blind leg                            |
| First step      | implement the detector                    | phase 0: rebase, field facts, height gate, pose and drift measurements                 |

## 11. Implementation (2026-10-13)

Scope, as directed: crater detection only. Phase 0 items, phase 2's long-term memory rules
and all of phase 3 are left to the team or to later.

### 11.1 What was built

Phase 1 of section 5.3, in the classifier both traversability nodes share.

| Change                                      | Where                                                                                                                                      |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| Scan origin kept with each scan             | `BufferedCloud::sensor_origin`, set in both nodes' point cloud callbacks                                                                   |
| Regional ground, `ground_ref`               | `compute_ground_reference` in `terrain_analysis.cpp`                                                                                       |
| Ray walk, `hole_rays`, `hole_depth`, `hole` | `compute_holes`                                                                                                                            |
| Hole cells are observed, not border         | `compute_border`                                                                                                                           |
| Hole cells are obstacles                    | `compute_obstacle`                                                                                                                         |
| One-sighting confirm                        | `hole_log_odds_hit` in `global_traversability`'s `_fuse_window`                                                                            |
| Parameters                                  | `hole_depth_m` 0.08, `hole_min_rays` 3, `hole_max_range_m` 4.0, `hole_reference_block_m` 0.5, `hole_log_odds_hit` 0.7 in `navigation.yaml` |
| Debug layers                                | `/local_layers/ground_ref`, `hole_depth`, `hole`                                                                                           |
| Tests                                       | `local_traversability/test/`: 16 gtest cases, a synthetic lidar, `hole_demo`                                                               |

### 11.2 Where the code differs from the plan in 5.3

1. **The ground reference is a fitted plane, not a median of medians.** The plan was a
   median over 0.5 m blocks, a 3x3 median across blocks, then interpolation. That version
   passed every crater test and failed on slopes: on 20 degrees it produced more than 100
   false hole cells along the far edge of the lidar's view. A median is only right in the
   middle of its samples; at the edge of the view, and at the edge of every shadow, the
   reference was carried across flat while the ground fell away. The fix: each block's
   sample is the median height at the mean position of its ground-like cells, and each
   block fits a plane to its 3x3 neighbourhood, dropping up to two samples more than half
   of `hole_depth_m` off the plane. Cells blend the four nearest planes. The offline
   prototype of 7.2 never met this because the sim scans were on gentle ground and gated
   to 3 m.
2. **The ray walk continues at half of `hole_depth_m`**, not at zero and not at the full
   depth. At the full depth only the far wall is marked and the shadowed floor is not.
3. **Hole cells had to be taken out of `border`.** Not in the plan and essential: a
   shadowed cell with no returns is otherwise published as unknown, which is free.

### 11.3 Results

All on a synthetic lidar (0.733 m up, -52 to +7 degrees elevation, about one beam per
third of a degree, 2 cm range noise where stated). No Gazebo, no real data.

- Unit tests: 16 of 16 pass. Flat ground, slopes of 10 and 20 degrees, undulating ground
  and a boulder give no holes. The old `step_down` test alone finds nothing on a
  0.45 x 0.30 m crater at 2 m.
- Range at which a crater is still marked, 2 cm noise, `hole_depth_m` 0.08: 0.50 x 0.45 m
  steep 3.5 m; 0.45 x 0.30 m bowl 3.0 m; 0.40 x 0.15 m bowl 2.5 m. No hole cell off the
  crater in any run. This is 0.5 m further than the prototype in 7.2 for the two smaller
  craters, on cleaner data.
- Slopes: a 0.45 x 0.30 m crater at 2 m is found on -20 to +20 degrees with no false cells
  on the bare slope.
- Both node executables, `navigation.yaml`, scripted rover at 0.28 m/s driving at a
  crater: local costmap lethal from 3.5 m (deep crater) and 2.6 m (shallow); global
  costmap from 3.4 m and 2.0 m; the global map held both down to 0.75 m, the end of the
  run. This is validation test 4 of section 8, on synthetic data. The global node's
  `max_height_above_ground_m` was raised to 0.2 for these runs, because at 0.05 it flags
  about 90 unrelated cells of the synthetic ground with the hole test on or off.
- The global map is late on the shallow crater (2.0 m) because of `min_obstacle_cells: 4`.
  With `hole_log_odds_hit` 0.25 it was 1.7 m, so the fast confirm is worth 0.3 m there.
- Cost on a desktop, about 40 000 buffered points: +0.5 ms per local classification,
  +1.2 ms per global one.

### 11.4 Limits found

- Sharp troughs are flagged: 16 cm peak to trough over a 1.5 m wavelength gives false
  cells; 10 cm over 3 m gives none. A dip 1.0-1.5 m wide is flagged at 20 cm deep, not at
  12 cm. This is the dips-versus-craters question of section 9, now with numbers.
- A known crater viewed from beyond `hole_max_range_m` loses score on its far-wall cells
  (they are observed and read as free). The close-range clearing rule of 5.4 would stop
  that and was left out of this version.

### 11.5 How it was built and run

The dev shell in this session did not expose ROS Jazzy, so the two packages were built
with colcon into a scratch directory using the Nix store paths the workspace was last
built with, and the nodes were run from there. The workspace's own `build/` and `install/`
were not touched. The end-to-end harness (a Python node publishing ray-cast scans and TF)
is scratch code and is not in the repo.

### 11.6 Next

1. Gazebo again with craters in other places and other paths (carve_craters.py), and with
   the mission's bucket poses.
2. Real lidar bag, tune `hole_depth_m`.
3. Port to `origin/feat/autonomy-refactoring`: the classifier sources are identical there.

### 11.7 Gazebo run (2026-10-13)

Setup: `~/perseus_simulation` at `fbf4c39` (`feature/lunabotics-arena`, with the MID-360
pattern sampler), plus the cratered heightmap applied from scratch copies of the world and
`Moon_DEM` (the crater edits are in `stash@{0}` of that repo; the repo itself was not
changed). GUI on the Intel GPU, real-time factor 0.1. Rover pose from Gazebo ground truth
published as `odom -> base_link`, because the sim EKF has no yaw source (its `imu0` is
`/imu`; the bridge publishes `/livox/imu`). Bucket at lift 30, tilt -20.

Two faults found and fixed in `compute_ground_reference`, each with a unit test:

| Fault                                 | Effect                            | Fix                                                                                                        |
| ------------------------------------- | --------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| roof behind the rover taken as ground | about 600 false hole cells        | a cell at or above the lidar is not a ground sample                                                        |
| wall top (0.72 m) taken as ground     | about 100 false cells by the wall | keep only samples connected to the ground near the rover; ridge on the plane fit raised from 0.001 to 0.05 |

Ablation on a recorded 856-scan drive (frames with any hole cell off a crater): all three
rules 14; without the ridge 16 to 20; without the connectivity rule 44 (up to 26 cells).
A cell-spread rule and a slope clamp made no difference and were removed.

Result with the final code, 7 m drive with four turns past all five craters: every crater
marked, first at 2.3 to 3.2 m, all five still in `/costmap` at the end; no hole cell off a
crater in the live run. Table and caveats in the walkthrough ("In Gazebo").

Also seen: the sim rover could not pivot in place near crater 2 and stayed stuck until
moved by hand.

### 11.8 Second Gazebo run and the memory fix (2026-10-13, after reboot, NVIDIA GPU)

The "stuck near crater 2" above was the route, not the rover: the arena's center column is
at odom (2.92, -1.12) and the route ran into it. Routes are now checked against every
boulder, the column, the poles, the walls and the craters before driving, pivots are made
with the bucket raised (lift 0) and straights with it lowered (lift 30).

A 17 m loop through the obstacle field and back (run 1, commit `fbf1bf7`):

| Crater         | First in hole layer | First lethal in `/costmap` | Cells in `/costmap` at the end |
| -------------- | ------------------- | -------------------------- | ------------------------------ |
| 1: 0.50 x 0.45 | 2.7 m               | 2.7 m                      | 36                             |
| 2: 0.45 x 0.30 | 3.2 m               | 2.8 m                      | 19                             |
| 3: 0.40 x 0.15 | 2.5 m               | 2.2 m                      | 2                              |
| 4: 0.50 x 0.45 | 3.7 m               | 3.5 m                      | 0                              |
| 5: 0.45 x 0.40 | 2.5 m               | 2.5 m                      | 16                             |

**The global map forgot craters.** Crater 4 was held at 24 to 30 cells within 2 m and
faded to none by 5 m. From far away a crater's floor is hidden by its near rim, its cells
read as ground, and each such view subtracted `log_odds_miss`.

Fix, in `global_traversability.cpp` (`_fuse_window`, new persistent layer `hole_range`):
each cell remembers the furthest range it has been seen as a hole from. A "free" verdict
from further away than that is ignored; from that range or closer it counts as before, and
once the cell's log-odds reach zero the memory is dropped. No new parameter: the range is
learned per cell, so a shallow crater (seen from 2.5 m) and a deep one (3.5 m) each get
their own. Same loop again (run 2): all five craters still in `/costmap` at the end, with
38, 21, 13, 20 and 19 cells.

Cost of the fix: a false hole is as hard to clear as a real one. It clears only when the
rover sees that ground as free from as close as it was marked from.

**Stray hole cells, not yet explained.** Two logged seconds in each run:

- beside boulder 7, about 0.55 m beyond it towards the west wall, near odom (1.4, 2.85):
  18 cells in run 1, 24 in run 2. Same place both times, so it is systematic.
- beside boulder 2 in run 1 (20 cells), and at the north wall by the judge platform, odom
  (6.9, -1.6), in run 2 (161 cells, during a pivot with the bucket raised).

Not checked: whether these reached `/costmap` (they sit on or beside cells that are lethal
anyway), the mission's own bucket poses, craters in other places, real lidar data.

### 11.9 Stray holes beside walls and boulders (2026-10-13)

Cause, found by replaying the recorded loops offline and printing the block samples: in the
stray frames the ground reference near the walls sat 0.2 to 0.45 m too high. The samples
lifting it came from the top of the 0.5 m wall, the top of a large boulder, and the part of
a wall face that shows above whatever hides its foot. Each is flat, above the bed, and
passes for ground when the bed beside it is out of sight. The connectivity rule of 11.7 did
not stop them, because a wall whose foot is hidden by a varying amount rises smoothly from
block to block like a slope.

Fix: a block gives a ground sample only when at least half of its cells are ground-like
(`REFERENCE_MIN_FILL` 0.5, was 3 cells of 25). A wall top is one or two cells wide, so its
blocks no longer qualify.

Replay of the recorded drives, frames with any hole cell off a crater:

| Drive            | Before: frames, largest blob | After: frames, largest blob |
| ---------------- | ---------------------------- | --------------------------- |
| loop 1 (573)     | 7, 35 cells                  | 8, 8 cells                  |
| loop 2 (598)     | 18, 164 cells                | 11, 5 cells                 |
| 11.7 drive (856) | 14, 4 cells                  | 4, 1 cell                   |

Furthest detection of each crater was unchanged (3.1, 3.0, 2.6, 3.6 and 3.7 m). What is left
is small: three cells at 3.8 m range that are a real 9 cm dip in the bed, and a few cells on
the footprint of boulder 7.

Live loop with both fixes (run 3): all five craters marked and still in `/costmap` at the
end with 39, 21, 18, 24 and 19 cells; one logged second with 6 hole cells off a crater,
beside the south-west construction pole. Replayed, that drive has 3 stray frames of 747,
largest 11 cells.

No unit test covers this fix. Two attempts at a synthetic wall with a hidden foot did not
reproduce the fault, so the evidence is the replays above.

Unexplained: in runs 2 and 3 a lethal cell within 0.35 m of crater 3 appeared in `/costmap`
when the rover was 4.7 m away, before the hole test marked the crater at 2.5 m. It did not
come from the hole test.
