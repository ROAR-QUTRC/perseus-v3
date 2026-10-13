# Crater Detection & Avoidance: Agent Brief

Revised 2026-10-13. This replaces the original brief, which was written before the repo had been read and assumed a RANSAC ground fit, an unknown lidar and ROS 2 Humble. Those assumptions were wrong or have been superseded. The evidence behind every statement here is in `crater_detection_notes.md` (section numbers in brackets). Read that file before changing code.

## Goal

Keep the rover out of holes in the Lunabotics arena, using the sensors on the rover. The deliverable is hole cells marked lethal in the Nav2 costmaps early enough for the planner to route around them. Measuring a crater's size or centre is not needed.

This is an observed failure, not only a rule to satisfy: in a recent field test the rover got stuck or slowed in craters, and some of those were dug by the rover itself while pivoting [1, 3].

## The two kinds of hole

| Kind          | Made by                                                       | Size                                                           | When it exists                          | Can the lidar see it?                                                   |
| ------------- | ------------------------------------------------------------- | -------------------------------------------------------------- | --------------------------------------- | ----------------------------------------------------------------------- |
| Arena crater  | arena staff                                                   | up to 40-50 cm wide and deep; at least three, random placement | before the run                          | yes, from about 2-3 m [7.2, 7.4]                                        |
| Self-made rut | the rover's wheels when pivoting, and the bucket when digging | not measured                                                   | appears during the run, under the rover | mostly not: too narrow or too shallow, and made in the blind zone [4.7] |

Avoidance does not need to know which kind a hole is. A hole that bogs the rover is a hazard either way [4.4].

## Confirmed facts about the system

- ROS 2 Jazzy, Nav2, Orange Pi. Speed cap 0.28 m/s.
- Lidar: Livox MID-360, 0.733 m up, pitched 45 deg down. It sees ground only in a forward wedge.
- RealSense D455: depth and point cloud are off; the infra pair feeds ORB-SLAM3. Do not plan on depth.
- Terrain is classified by one shared classifier (`nav-stack/local_traversability`, `terrain_analysis.cpp`) run by two nodes on a 0.1 m grid over the last 1 s of raw scans:
  - `local_traversability`: 8 m window, 10 Hz, forgets everything each cycle.
  - `global_traversability`: remembers a verdict per cell as log-odds, for the whole run.
- The existing negative-obstacle test (`max_step_down_m: 0.15`) misses most craters, because the slope allowance absorbs the drop and the crater interior is shadow, which the stack treats as free [4.2].
- Ground closer than about 1.4-1.5 m from the sensor is never classified (`min_range_m` 1.6 and 1.7) [4.5].
- Every costmap is in the `odom` frame. `odom` is two dead-reckoning sources fused by an EKF (BIEVR-LIO and ORB-SLAM3). Nothing corrects it: ORB-SLAM3 loop closing is off, and the ArUco fix (`/arena/localise`) is an on-demand service that only moves `map -> odom` [3].
- The mission is a loop: excavation, dig, construction, dump, back to excavation. The rover turns round at each end, at similar places every cycle [2.2].

## Start here: the base branch

The rest of autonomy is on `origin/feat/autonomy-refactoring`, 176 commits ahead of `feat/detect-crater`, which has no commits of its own. The planner, ORB-SLAM3, the bucket mission and the range gates described above are all on that branch. **Do crater work on top of it.** Confirm with Kelvin whether to branch from it or wait for it to merge [2.2].

## The approach

Settled. Do not reopen without new evidence.

1. **One detector for all holes, inside the shared classifier.** A cell is a hole when lidar rays are observed travelling below where the ground should be. The ground reference is a regional median of ground-like cells; each return more than `hole_depth_m` below it marks the cells its ray crossed while under the reference; a cell marked by `hole_min_rays` rays is a hole [5.3].
2. **Detect live on every pass.** The live scan is what keeps the rover out of a hole. Nothing depends on a map built earlier in the run.
3. **Trust memory over seconds, doubt it over minutes.** Short-term memory in the global grid carries a hole across the near blind zone and is required. Long-term memory helps the planner choose a route and corrects itself when the lidar looks again [4.5, 5.4].
4. **Ruts need more than the lidar.** Prevention (the pivot-avoidance work on the branch) and marking where the rover pivoted or slipped [5.5].

Starting parameters: `hole_depth_m` 0.08, `hole_min_rays` 3, `hole_max_range_m` 4.0, `hole_log_odds_hit` 0.7.

## Work packages

### Phase 0: unblock (no detector code)

1. Move to the base branch (above).
2. Ask the team what bogged the rover: which holes, how wide and deep, where in the cycle.
3. Fix the height gate. `filter_scan` drops points by height in the tilted sensor frame, which discards forward ground beyond about 2.85 m. It limits rock detection too. Confirm on the real rover first [7.5].
4. Measure in sim how much ground the lidar sees with the bucket in the two poses the mission uses: travel (lift 25, tilt -20) and carry (lift 0, tilt -28). Lift 0 was blind dead ahead in earlier tests, and the loaded leg is driven in it [4.6].
5. Measure how far `odom` drifts in x/y over one mission cycle [8, test 6].

### Phase 1: the live detector

**Done on 2026-10-13 and committed on `feat/detect-crater`.** Tested with unit tests, against a synthetic lidar, and in Gazebo on the cratered arena with the rover driving (notes 11.7 to 11.9); not yet run on real data. Read `crater_detection_README.md` for the summary and parameters, `crater_detection_walkthrough.md` for how it works, and notes section 11 for results and where it differs from the list below (the ground reference is a robust plane fit, not a median of medians). The global map's rule for keeping holes (Phase 2, first bullet) is also done, by range: notes 11.8. What remains of this phase is a real bag and porting to the base branch.

- `terrain_analysis.hpp/.cpp`: store the sensor origin with each buffered scan; add the ground-reference pass and the ray pass; add the parameters.
- OR the result into the existing `obstacle` layer, and keep it as its own `hole` layer.
- `global_traversability.cpp`: a hole verdict confirms in one update. The shallowest crater is only visible for about 0.5 m of travel, and the existing confirmation takes about 1.5 s.
- Debug layers `ground_ref`, `hole_depth`, `hole`.
- `navigation.yaml`: the parameters, with the measurements behind them in comments, as the file does for every other threshold.

### Phase 2: memory

- Hole cells lose log-odds only when seen flat from within about 2.5 m.
- No other machinery until drift has been measured. The action depends on the result: under about 0.15 m nothing more is needed; up to about 0.5 m, age unrefreshed holes down to a non-lethal cost; beyond that it is a localisation problem for Kelvin [5.4].

### Phase 3: self-made ruts

Proposals that touch the planner and mission tree. Agree them with Kelvin before building.

- Stamp a non-lethal cost disc (about 0.75 m radius) where the rover pivots.
- Do the same where `/watchdog/mobility_status` reports slip. Nothing uses that signal today.
- Check the effect on `arena_server` waypoint choice (`waypoint_max_cost` 50) first.

### Optional

- Tag each hole cell with when it was first seen and whether the rover had already driven over it. This is the useful part of the "baseline survey" idea: it counts new holes per cycle, which measures whether pivot avoidance works. It is a diagnostic only [5.6].

## Do not

- Do not use RANSAC, per-patch plane fits or ML for detection [4.1].
- Do not add a separate Nav2 costmap layer. The classifier's `obstacle` layer already reaches both costmaps.
- Do not lower `hole_depth_m` below about 0.07 to catch ruts. At 0.06 it flags natural dips [7.4].
- Do not build avoidance on a reference map from the first traverse [5.6].
- Do not move the costmaps into the `map` frame as part of this work [5.4].
- Do not treat unobserved cells as obstacles to catch crater shadow. Three linked settings depend on unknown being free (`UNMAPPED` in `navigation.yaml`).
- Do not read `/Laser_map`. It is for visualisation; z-wander makes it unusable as a ground reference.
- Do not mask the excavation zone.

## Validation

Full list in the notes [8]. The tests that gate each phase:

| Gate    | Test                                                                              | Pass                                                                                                 |
| ------- | --------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Phase 1 | sim arena without craters                                                         | zero hole cells on the bumpy bed and around boulders                                                 |
| Phase 1 | sim arena with carved craters, rover driving at 0.28 m/s, several approach angles | every 40-50 cm crater detected by 2 m                                                                |
| Phase 1 | drive straight at a crater                                                        | the hole stays lethal in `/costmap` until the rover has passed, including inside the near range gate |
| Phase 1 | replay `rock_nav_test_2`                                                          | zero lethal cells under the driven path                                                              |
| Phase 2 | inject an `odom` offset between seeing a crater and returning                     | stale cells clear and the real crater is marked before the rover reaches it                          |
| Phase 3 | real rover: pivot 360 deg in sand, measure the rut, approach from 5 m             | recorded either way; decides how much of phase 3 is needed                                           |
| All     | real 45 x 45 cm and 40 x 15 cm holes in sand, approached from 5 m                 | detected by 2 m                                                                                      |

Sim results bound the geometry, not the noise. The simulated lidar is a noiseless fixed-ring scanner; thresholds have to be confirmed on real data.

## Simulation

`~/perseus_simulation`, branch `feat/arena-craters` (off `feature/lunabotics-arena`, uncommitted), with five craters carved into the arena heightmap. `ROS_DOMAIN_ID=51`, `rmw_cyclonedds_cpp`, run tools through `scripts/pixi-run.sh` [6, 7.1]. All sim work so far was with the rover stationary and on the older configuration [7.6].

## Open questions for the team

- What bogged the rover, and how big was it?
- Kelvin: is ORB-SLAM3 loop closing meant to be on? It is off, and the odometry node discards the pose jumps it would produce.
- Kelvin: should `/arena/localise` be called during a mission? Nothing calls it now.
- Kelvin: branch from `feat/autonomy-refactoring` now, or wait for the merge?
- Can the load be carried lower than lift 0?
- Are soft-cost pivot and slip marks acceptable?
- Where is the `rock_nav_test_2` bag?

## Who owns what

- Crater detection and its memory rules: this work.
- Rock detection testing, planner, controllers, localisation, mission tree: Kelvin.
- The height gate fix sits in the shared classifier and affects both. Coordinate it.

## Background references (from the original brief; none adopted)

Kept for context. The repo's own classifier covers what these packages would have provided, and the IRI packages and `depth_nav_tools` are ROS 1. Descriptions came from search results, not from running the code.

- Elevation mapping: [FastDEM](https://github.com/Ikhyeon-Cho/FastDEM), [grid_map_pcl](https://index.ros.org/p/grid_map_pcl/), [ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping) (unmaintained), [height_mapping](https://github.com/Ikhyeon-Cho/height_mapping).
- Negative obstacles: [iri_point_cloud_hole_detection](https://gitlab.iri.upc.edu/labrobotica/ros/navigation/3d_navigation/iri_point_cloud_hole_detection/-/commit/86f0e8ee6d6a224a2732ec4a72a25d3340e97509), [iri_lidar_obstacle_detection](https://gitlab.iri.upc.edu/labrobotica/ros/navigation/3d_navigation/iri_lidar_obstacle_detection/-/commit/b65e36e486747f0ab3aca2c9b5a8d5afbf30920c.diff), [iri_obstacle_detection_normals](https://gitlab.iri.upc.edu/labrobotica/ros/navigation/3d_navigation/iri_obstacle_detection_normals/-/commit/0203614b837bf1e55bbe4640b0b85e4c30007c9a.diff), [depth_nav_tools](https://github.com/mdrwiega/depth_nav_tools).
- Circle and ellipse fitting, only relevant if crater-based localisation is ever wanted: [circle_detection](https://github.com/josafatburmeister/circle_detection), [sdg002/RANSAC](https://github.com/sdg002/RANSAC), [AkosSeres/ellipse_detect](https://www.github.com/AkosSeres/ellipse_detect).
- Lunabotics reference: [CMU Lunar ROADSTER lab report](https://www.contrib.andrew.cmu.edu/~boxiangf/mrsd/TeamI_sjdsouza_ILR04.pdf) (depth threshold plus OpenCV on an occupancy image).
- Test data: [CSIRO negative obstacle approach dataset](https://researchdata.edu.au/ground-vehicle-approaching-negative-obstacles/1607457).
- Competition rule note: the 2026 FAQ allows natural terrain features such as craters and rocks to be used for localisation.
