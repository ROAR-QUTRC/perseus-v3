# Crater detection: what this branch adds and how to test it

Branch `feat/detect-crater`. Written 2026-10-13 for teammates who will test it.

## What it does

The rover now marks craters as lethal in both Nav2 costmaps, using the lidar it already
has. Nothing in the detector knows where a crater, a wall or a zone is: it works on the
scan alone, so craters can be anywhere.

- A crater 40-50 cm wide is marked from about 2.5-3.5 m away.
- Once marked, it stays in the global costmap after the rover drives past it or away from
  it.
- Hole cells go into the classifier's existing `obstacle` layer, so the planner treats
  them exactly as it treats rocks. There is no bounding box and no new costmap layer.

Tested with unit tests, on a synthetic lidar, and in Gazebo on the arena with five carved
craters. **Not yet tested on the real rover or on real lidar data.**

## How it works, in short

1. **The old test misses craters.** From the mast, a crater's near rim hides its floor.
   The floor cells get no returns, and the stack treats "no returns" as free.
2. **Use the beam, not only its end.** A return on the crater's far wall proves the path
   to it was empty. Part of that path is below ground level. Empty space where ground
   should be is a hole.
3. **Estimate where the ground should be** (`compute_ground_reference`). The grid is cut
   into 0.5 m blocks. Each block that is at least half ground gives one height sample, and
   a plane is fitted through each block's neighbourhood. A crater cannot drag this down
   into itself.
4. **Walk each low return back along its beam** (`compute_holes`). Every cell the beam
   crossed while under the ground gets a count. Three beams make the cell a hole.
5. **A hole is an obstacle.** It is also no longer "unknown", which is what used to let
   the planner drive into the shadow.
6. **Remember it** (`global_traversability`). One sighting marks a hole in the global map.
   A later "looks free" view only counts against it if taken from as close as the hole was
   seen from, because from further away every crater looks like flat ground.

## Files changed

All under `software/ros_ws/src/nav-stack/`.

| File                                                                        | What changed                                            |
| --------------------------------------------------------------------------- | ------------------------------------------------------- |
| `local_traversability/src/local_traversability/terrain_analysis.cpp`        | the detector: ground reference and ray walk             |
| `local_traversability/include/.../terrain_analysis.hpp`                     | new parameters, new layers, sensor position per scan    |
| `local_traversability/src/local_traversability/local_traversability.cpp`    | reads the parameters, publishes the debug layers        |
| `global_traversability/src/global_traversability/global_traversability.cpp` | reads the parameters, one-sighting confirm, hole memory |
| `autonomy_bringup/config/navigation.yaml`                                   | the parameters, with the measurements behind them       |
| `local_traversability/test/`                                                | unit tests, a synthetic lidar, and the `hole_demo` tool |

## Parameters

All of them are in `autonomy_bringup/config/navigation.yaml`. The four `hole_*` detector
parameters appear twice, once under `global_traversability:` and once under
`local_traversability:`. **Keep the two copies the same**, or the two costmaps will
disagree about what is a hole.

| Parameter                | Node   | Value | What it is                                                                   |
| ------------------------ | ------ | ----- | ---------------------------------------------------------------------------- |
| `hole_depth_m`           | both   | 0.08  | how far below the expected ground a return must be to count as hole evidence |
| `hole_min_rays`          | both   | 3     | how many separate beams must cross a cell under the ground to call it a hole |
| `hole_max_range_m`       | both   | 4.0   | returns further than this from the lidar are not used as hole evidence       |
| `hole_reference_block_m` | both   | 0.5   | size of the blocks the expected ground is estimated over                     |
| `hole_log_odds_hit`      | global | 0.7   | how much one hole sighting adds to a cell in the global map                  |

### What each one does when you change it

| Parameter                | Raise it and...                                   | Lower it and...                                  |
| ------------------------ | ------------------------------------------------- | ------------------------------------------------ |
| `hole_depth_m`           | shallow craters are missed and range drops        | natural dips in the bed start to count as holes  |
| `hole_min_rays`          | fewer false holes from stray returns; range drops | a few bad returns can make a hole                |
| `hole_max_range_m`       | noise at long range can make false holes          | craters are marked later                         |
| `hole_reference_block_m` | the expected ground stops following real bumps    | a crater becomes its own ground and is not seen  |
| `hole_log_odds_hit`      | no effect above `log_odds_occupied` (0.5)         | below 0.5 the global map needs several sightings |

- **Tune `hole_depth_m` first.** It is the main dial. In Gazebo 0.06 flagged a natural dip
  in the bed; 0.08 did not. Do not go below about 0.07.
- **`hole_depth_m: 0.0` switches the whole test off.** Use this to compare with and
  without.
- **`hole_reference_block_m` should stay at about the width of the largest crater.**

Three existing parameters also affect what reaches the costmap:

| Parameter            | Node   | Value             | Effect on craters                                                      |
| -------------------- | ------ | ----------------- | ---------------------------------------------------------------------- |
| `min_obstacle_cells` | both   | 4 global, 2 local | a hole blob smaller than this is dropped as noise                      |
| `log_odds_miss`      | global | 0.4               | how fast a marked cell is cleared when later seen free from close by   |
| `min_range_m`        | both   | 1.0               | nothing closer than this is classified; the global map covers that gap |

### How to set them

The nodes read their parameters once, at start-up. `ros2 param set` on a running node will
be accepted but will change nothing.

1. Edit `software/ros_ws/src/nav-stack/autonomy_bringup/config/navigation.yaml` (both
   copies of the `hole_*` lines).
2. Rebuild so the installed copy is updated, then relaunch:

   ```shell
   colcon build --packages-select autonomy_bringup
   ```

   With `colcon build --symlink-install` the rebuild is not needed; just relaunch.

To try a value once without editing the file, override it when starting a node by hand:

```shell
ros2 run local_traversability local_traversability --ros-args \
  --params-file src/nav-stack/autonomy_bringup/config/navigation.yaml \
  -p hole_depth_m:=0.10
```

## How to build and test

From `software/ros_ws`, in the environment you normally build in:

```shell
colcon build --packages-select local_traversability global_traversability autonomy_bringup
colcon test --packages-select local_traversability --ctest-args -R test_hole_detection
colcon test-result --verbose
```

18 tests should pass. To see the detector on one synthetic crater (width, depth, distance,
and optionally `hole_depth_m`):

```shell
build/local_traversability/hole_demo 0.5 0.45 2.5        # deep crater, 2.5 m ahead: found
build/local_traversability/hole_demo 0.4 0.15 3.0        # shallow one, too far: not found
build/local_traversability/hole_demo 0.4 0.15 3.0 0.05   # found with a lower threshold
```

## What to look at on the rover or in the sim

Add these to RViz as Map displays, fixed frame `odom`:

| Topic                      | What to look for                                                    |
| -------------------------- | ------------------------------------------------------------------- |
| `/local_layers/hole`       | the verdict: the crater's cells, and nothing else                   |
| `/local_layers/hole_depth` | where beams went under the ground, and by how much                  |
| `/local_layers/ground_ref` | the expected ground: smooth and continuous straight across craters  |
| `/local_costmap_terrain`   | hole cells lethal                                                   |
| `/costmap`                 | the crater still there after the rover has driven up to and past it |

Reading them:

- **`hole` empty where `hole_depth` is bright:** too few beams. Look at `hole_min_rays`.
- **`hole_depth` empty over a crater:** look at `ground_ref`. It has probably dipped into
  the crater, or it is missing there.
- **Holes where there is no crater:** look at `ground_ref` there. If it is well above the
  real ground, something that is not ground was taken as ground. Please record a bag.

## Test results so far

Gazebo, cratered arena, rover driving a 17 m loop through the obstacle field at 0.28 m/s,
pivoting with the bucket raised and driving with it lowered (final code, 2026-10-13):

| Crater (width x depth) | First in hole layer | First lethal in `/costmap` | Cells in `/costmap` at the end |
| ---------------------- | ------------------- | -------------------------- | ------------------------------ |
| 1: 0.50 x 0.45 m       | 2.6 m               | 2.3 m                      | 39                             |
| 2: 0.45 x 0.30 m       | 3.0 m               | 2.7 m                      | 21                             |
| 3: 0.40 x 0.15 m       | 2.5 m               | see note                   | 18                             |
| 4: 0.50 x 0.45 m       | 3.7 m               | 3.4 m                      | 24                             |
| 5: 0.45 x 0.40 m       | in view at start    | in view at start           | 19                             |

- Craters 1 and 5 were already in view when the nodes started, so their first distances
  are where the rover happened to be.
- Crater 3: a lethal cell near it appeared in `/costmap` when the rover was 4.7 m away,
  before the hole test marked it at 2.5 m. That early cell did not come from the hole test
  and has not been explained.
- Hole cells away from any crater: one logged second in the run, 6 cells, beside a
  construction-zone pole.

## Known limits

- **Not run on real lidar.** The simulated lidar has no range noise. `hole_depth_m` has to
  be confirmed on real scans.
- **The bucket blocks the lidar.** With the bucket at its start-up pose (lift 0) a crater
  dead ahead was not seen at all. With it lowered to lift 30 it was. The mission's own
  travel and carry poses have not been tested.
- **Shallow craters are seen late.** A 15 cm deep crater shows from about 2.5 m.
- **Broad dips count as holes** once they are sharp enough: a dip 1.0-1.5 m wide is
  flagged at 20 cm deep but not at 12 cm.
- **A few stray cells remain.** On recorded drives, 0.4-1.8% of frames have a hole cell
  away from any crater, at most 11 cells, usually beside a boulder, a pole or a wall.
- **A false hole is as sticky as a real one.** It clears only when the rover sees that
  ground as free from as close as it was marked from.
- **Ruts the rover digs itself are not detected.** They are too narrow or too shallow, and
  are made under the rover where the lidar cannot see.
- **The global map is in `odom`.** If odometry drifts, remembered craters drift with it.
  The live scan still marks a crater where it really is on every approach.
- **Craters were tested in five fixed places.** Moving them and re-running is still to do.
- **This branch is behind `feat/autonomy-refactoring`.** The classifier files were the
  same on both when this was written, so the change should port directly.

## Commits on this branch

1. `fbf1bf7`: the detector, parameters, debug layers and tests.
2. `f5ccdbb`: the global map keeps craters when they are seen from far away.
3. The commit that adds this file: fewer false holes beside walls and boulders, and the
   documents.
