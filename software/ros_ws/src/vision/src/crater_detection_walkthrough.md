# Crater detection: how it works, step by step

A walkthrough of the crater (hole) test added to the terrain classifier on 2026-10-13. It
is written to be read top to bottom with the code open beside it. Background and the
reasoning behind the plan are in `crater_detection_notes.md`; this file is about the code
that now exists.

Scope: detecting craters and getting them into the costmaps. Nothing here touches the
planner, localisation, the bucket or the mission tree.

## The one-paragraph version

A crater seen from the rover's mast is mostly invisible: its own near rim hides its floor.
What the lidar does see is a few returns on the far wall, lower than the ground around
them. Each of those returns sits at the end of a beam that travelled in a straight line
from the sensor, so the beam must have passed _under ground level_ on its way there. The
test finds those beams and marks every cell they passed under ground in. Enough beams
through a cell make it a hole, and a hole is a lethal obstacle.

## Where the code is

Everything is in the classifier both traversability nodes already share.

| What                               | File (under `nav-stack/`)                                                   | Function / symbol                       |
| ---------------------------------- | --------------------------------------------------------------------------- | --------------------------------------- |
| Step 3: where the ground should be | `local_traversability/src/local_traversability/terrain_analysis.cpp`        | `compute_ground_reference`, `fit_plane` |
| Step 4: the ray walk               | same file                                                                   | `compute_holes`                         |
| Step 5: verdict into the costmap   | same file                                                                   | `compute_border`, `compute_obstacle`    |
| Order of the passes                | same file                                                                   | `classify_terrain`                      |
| Parameters and scan origin         | `local_traversability/include/.../terrain_analysis.hpp`                     | `TerrainParameters`, `BufferedCloud`    |
| Step 6: memory                     | `global_traversability/src/global_traversability/global_traversability.cpp` | `_fuse_window`                          |
| Values, with the measurements      | `autonomy_bringup/config/navigation.yaml`                                   | `hole_*` under both nodes               |
| Tests                              | `local_traversability/test/test_hole_detection.cpp`                         | 18 tests                                |
| Synthetic lidar                    | `local_traversability/test/synthetic_scan.hpp`                              | `scan`, `with_bowl_crater`, ...         |
| A program to watch it work         | `local_traversability/test/hole_demo.cpp`                                   | `hole_demo`                             |

## Step 1. Why the test that was already there misses craters

The classifier builds a 0.1 m grid from the last second of lidar points. For each cell it
keeps the lowest return (`elevation`) and compares it with the cells around it. The
existing negative-obstacle test, `max_step_down_m`, asks: is this cell's floor much lower
than its neighbours' floors?

That needs returns from inside the hole. Here is what the lidar gets from a 0.5 m crater
2.5 m ahead, one row of cells along the centre line (output of `hole_demo`, first two
columns):

```
   x [m]  returns  lowest [m]
    2.25       11       -0.00
    2.35        0         nan     <- inside the crater
    2.45        0         nan     <- inside the crater
    2.55        0         nan     <- inside the crater
    2.65        3       -0.13     <- inside the crater
    2.75       54       -0.15     <- inside the crater (the far wall)
    2.85       12       -0.00
```

Three of the five crater cells have no returns at all. The two that do read 13-15 cm low,
although the crater is 45 cm deep, because the beams only reach the upper part of the far
wall. `max_step_down_m` allows a slope of 40 degrees between neighbours before it counts a
drop, which is 8.4 cm per cell, so a 15 cm drop two cells from the rim passes as a slope.
Result: zero cells flagged. And cells with no returns are published as unknown, which this
stack treats as free.

So the old test fails twice: the cells that matter are empty, and the ones that are not
empty do not look deep.

## Step 2. The idea: use the beam, not just where it ended

```
   lidar *
          \  .
           \    .                         beams that clear the near rim
            \      .
             \        .
   ___________\__________.______              ___________________  ground level
        near rim         \  .    \           /
                          \     . \_________/ <- far wall: the only returns
                    floor in shadow
```

A lidar return tells you two things. The obvious one is "there is a surface here". The
other is "there is nothing between here and the sensor", because the beam got through.

For a far-wall return, part of that empty stretch lies below ground level. Empty space
where ground should be is a hole. This is direct evidence, not an inference from missing
data: it cannot be produced by a sparse scan, a shadow behind a rock, or the edge of the
field of view, because all of those give _no_ returns rather than a low one.

To use it, the code needs two things it did not have before:

1. **Where each scan was taken from.** `BufferedCloud` now stores `sensor_origin` next to
   the points. It is stored per scan because the rover moves during the one-second buffer.
2. **Where the ground "should" be**, so it can tell when a beam is below it. That is step 3.

## Step 3. Where should the ground be? (`compute_ground_reference`)

This is the hard part. The answer has to follow real slopes and bumps in the bed, but must
_not_ follow the ground down into a crater, or the crater would look flat.

The classifier already has a `ground` layer, but it cannot be used: it is built to follow
the lowest nearby floor, which is right for finding rocks and exactly wrong here.

The new layer, `ground_ref`, is built in four stages.

**Stage 1: one sample per 0.5 m block.** The grid is cut into blocks of 5 x 5 cells. Each
block gives one sample: the _median_ height of its ground-like cells, placed at the
_average position_ of those cells. Cells the rock tests already flag are left out, so a
wall does not lift the result. A median ignores a few odd cells in a way an average does
not.

Only cells below the lidar can be samples: ground is something the lidar looks down on,
and a roof is as flat as the bed.

**Stage 2: keep only the ground the rover is standing on.** Flat is not enough to be
ground. The top of the arena wall is flat too. What tells them apart is that ground joins
the ground under the rover without a jump. So the median of the samples within 2.5 m of
the lidar sets the starting height, and the ground is grown outwards from there: a sample
is accepted only if an accepted sample within two blocks is at nearly its height (within
0.23 m: half a metre of 20 degree slope plus 5 cm). Everything else is dropped.

**Stage 3: fit a plane to each block's neighbourhood.** Each block takes its own sample
and its eight neighbours' (up to nine points over 1.5 m) and fits a plane through them.
Then it is made robust by hand: find the sample furthest from the plane; if it is more
than 4 cm off, drop it and fit again; do that at most twice. A crater can drag down one
block, or two if it sits across their boundary, and this throws those out. It is
deterministic, which is why this is not RANSAC.

**Stage 4: back to cells.** Each cell reads the planes of the four nearest blocks at its
own position and blends them, so there are no seams between blocks.

### A mistake worth learning from

The first version did not fit planes. It used the median of the nine block medians, then
interpolated. All the crater tests passed. The slope test failed: on a 20 degree slope it
invented more than 100 hole cells, all along the far edge of what the lidar could see.

The cause: a median (or an average) is only correct in the _middle_ of the samples it is
taken over. At the edge of the view the samples are all on one side, so the reference was
carried across flat while the real ground kept falling away, and the ground itself then
looked like it was under the reference. A shadow behind a rock, or behind a crater, makes
the same kind of edge, so this would have shown up everywhere on a sloped bed.

A plane has a slope, so it extends correctly past its samples. That is the whole reason
for the plane fit, and also why each sample carries the position of the cells it came
from rather than its block's centre. `SlopesHaveNoHoles` is the test that guards this.

## Step 4. The ray walk (`compute_holes`)

For every point in the buffer:

1. Look up `ground_ref` in the point's cell. If the point is not more than `hole_depth_m`
   (8 cm) below it, skip. Nearly every point stops here, so the pass is cheap.
2. Skip it if it is further than `hole_max_range_m` (4 m) from the sensor along the
   ground. Out there the far-wall drop of a real crater is no bigger than the error in
   `ground_ref`.
3. Otherwise walk back from the point toward that scan's `sensor_origin` in half-cell
   steps. At each step, compare the beam's height with `ground_ref` in the cell it is
   over. While the beam is more than half of `hole_depth_m` below, add one to that cell's
   `hole_rays` (once per beam per cell) and record the depth in `hole_depth`. Stop when
   the beam comes back up to ground level. That happens at the near rim.

The walk is what marks the shadowed floor. In the table from step 1, the three empty
cells get 12, 21 and 25 beams through them.

Why the walk continues at half the starting depth: the return is the deepest point of the
beam, and every cell nearer the sensor sees the same beam higher up. Holding them to the
full 8 cm would mark only the far wall and leave the middle of the crater unmarked.

## Step 5. From evidence to the costmap

Three small changes turn `hole_rays` into something Nav2 avoids.

1. **Verdict.** `hole` is 1 where `hole_rays >= hole_min_rays` (3). One stray low return
   cannot make a hole.
2. **A hole cell is "observed".** `compute_border` used to call any cell with no returns
   "border", which is published as unknown and treated as free. A crater floor has no
   returns, so without this change the hole would be thrown away one function later. Now a
   hole cell is never border. This is the easiest part to miss and the tests check it.
3. **A hole cell is an obstacle.** `compute_obstacle` ORs `hole` in with the existing
   tests. From there nothing is new: small blobs are pruned (`min_obstacle_cells`), the
   result is inflated by the robot radius, and it is published as cost 100.

The same picture after all five steps:

```
   x [m]  returns  lowest [m]  ground_ref [m]  rays below  verdict
    2.25       11       -0.00           -0.00           0  -
    2.35        0         nan           -0.00           0  -
    2.45        0         nan           -0.00          12  HOLE
    2.55        0         nan           -0.00          21  HOLE
    2.65        3       -0.13           -0.00          25  HOLE
    2.75       54       -0.15           -0.00          22  HOLE
    2.85       12       -0.00           -0.00           0  -

  Top view (O = hole, . = ground seen, blank = no returns), 0.1 m per character

  ..........OOO........
  ......... OOOO.......
  ......... OOOO.......
  ..........OOO........
```

`ground_ref` stays at 0 straight across the crater, which is step 3 doing its job. The
cell at 2.35 m, just inside the near rim, is not marked: beams pass over it above ground
level. The robot-radius inflation covers it.

## Step 6. Remembering it (`global_traversability`)

`local_traversability` rebuilds everything ten times a second and remembers nothing.
`global_traversability` keeps a score per cell (log-odds) that goes up when a cell is seen
as an obstacle and down when it is seen as free; above 0.5 the cell is lethal.

For rocks the step up is `log_odds_hit: 0.25`, so a rock needs three sightings. That was
tuned to reject flickering noise. A crater cannot wait that long. It only becomes visible
at 2-3 m, and once it is closer than `min_range_m` the lidar stops classifying it, so the
global map is the only thing still holding it. So a hole verdict gets its own step,
`hole_log_odds_hit: 0.7`, which crosses 0.5 in one sighting.

No other memory rule was added. Unobserved cells already keep their value, which is what
carries the crater while the rover is on top of it.

### Not forgetting it

The first version forgot craters as the rover drove away: one held at 30 cells faded to
none by 5 m. From far away a crater's floor is hidden behind its near rim, its cells read
as ground, and every such view voted it back to free.

So each cell also remembers the furthest range it was seen as a hole from (a layer called
`hole_range`). A "free" verdict from further away than that is ignored. From that range or
closer it counts as before, so a hole that really has gone can still be cleared. There is
no parameter: a shallow crater that showed from 2.5 m and a deep one that showed from 3.5 m
each keep their own range.

## The dials

| Parameter                | Value | Raise it and...                           | Lower it and...                                  |
| ------------------------ | ----- | ----------------------------------------- | ------------------------------------------------ |
| `hole_depth_m`           | 0.08  | shallow craters are missed; range drops   | natural dips in the bed start to count as holes  |
| `hole_min_rays`          | 3     | more robust to stray returns; range drops | a few bad returns can make a hole                |
| `hole_max_range_m`       | 4.0   | noise at long range can make false holes  | less warning                                     |
| `hole_reference_block_m` | 0.5   | the reference stops following real bumps  | a crater becomes its own ground and vanishes     |
| `hole_log_odds_hit`      | 0.7   | no effect above 0.5                       | below 0.5 the global map needs several sightings |

`hole_depth_m: 0.0` switches the whole test off. `hole_depth_m` is the one to tune first,
and it has to be tuned on real scans.

## Try it yourself

Build and test (from `software/ros_ws`, in the environment you normally build in):

```shell
colcon build --packages-select local_traversability global_traversability
colcon test --packages-select local_traversability --ctest-args -R test_hole_detection
colcon test-result --verbose
```

Watch it on one crater. Arguments are width, depth, distance, and optionally
`hole_depth_m`:

```shell
build/local_traversability/hole_demo 0.5 0.45 2.5        # deep crater, 2.5 m ahead
build/local_traversability/hole_demo 0.4 0.15 2.0        # shallow one, found
build/local_traversability/hole_demo 0.4 0.15 3.0        # same one, too far: not found
build/local_traversability/hole_demo 0.4 0.15 3.0 0.05   # found again with a lower threshold
```

Good exercises: move a crater out until it disappears and check the distance against
`width x 0.73 / distance = hole_depth_m`; then change `RAY_WALK_DEPTH_FRACTION` to 1.0,
rebuild, and see which test fails and why.

On the rover or in Gazebo, add these to RViz as Map displays, in this order:

| Topic                      | What to look for                                                |
| -------------------------- | --------------------------------------------------------------- |
| `/local_layers/elevation`  | craters show as gaps with a low strip on the far side           |
| `/local_layers/ground_ref` | should look smooth and continuous straight across those gaps    |
| `/local_layers/hole_depth` | bright where beams went under the ground, and by how much       |
| `/local_layers/hole`       | the verdict                                                     |
| `/local_costmap_terrain`   | hole cells lethal, with the inflation ring around them          |
| `/costmap`                 | the same crater still there after the rover has driven up to it |

If `hole` is empty where `hole_depth` is bright, `hole_min_rays` is the cause. If
`hole_depth` is empty over a crater, look at `ground_ref`: it has probably dipped.

### Two more, found in Gazebo

The synthetic lidar only ever saw ground, rocks and craters. The Gazebo arena has a
building around it, and the first drive there produced hundreds of false hole cells.

1. **The roof.** The lidar is pitched down at the ground ahead, so behind the rover it
   looks up, at a roof 4 to 5 m overhead. A roof is flat, so it passed as "ground-like",
   the reference behind the rover jumped to roof height, and every ray under it counted as
   a hole: about 600 cells. Fix: a cell at or above the lidar cannot be ground.
2. **The wall top.** The arena wall is 0.72 m high with a flat top, just under the lidar.
   It lifted the reference beside the wall by 0.3 m and the good ground along the wall
   read as a hole: about 100 cells. Fix: stage 2 above. The plane fit was also given a
   real ridge penalty (0.05 m^2), because three samples nearly in a line gave it a wild
   slope.

Both are the same lesson: the detector is only as good as its idea of where the ground
is, and "flat" does not mean "ground". Each has a unit test
(`CeilingBehindTheRoverIsNotGround`, `WallTopBesideTheRoverIsNotGround`).

Two further rules were tried and then removed (a limit on the height spread inside a
cell, and a clamp on the fitted slope). A recorded drive replayed with and without them
showed no difference, so they were not kept. Nothing in the detector depends on where a
crater is.

### A third, found on the second Gazebo run

Near the walls the reference still sat up to 0.45 m too high in a few frames. The samples
came from the top of the wall, the top of a big boulder, and the upper part of a wall face
whose foot was hidden. A block used to need only 3 ground-like cells of its 25 to give a
sample, and a wall top is one or two cells wide. A block now needs **at least half** of its
cells to be ground-like (`REFERENCE_MIN_FILL`). On the recorded drives the largest false
blob went from 164 cells to 5, with the craters found from the same distances.

## What has been checked, and what has not

Checked, on 2026-10-13:

- **Compiles** with the repo's `-Werror` flags, both packages.
- **18 unit tests pass** against the synthetic lidar: flat ground, slopes of 10 and 20
  degrees both ways, undulating ground and a boulder give no holes; seven crater cases are
  found and are lethal in the final cost; the shadowed centre cell is marked; the old test
  alone finds nothing on the same crater.
- **Range**, synthetic lidar, flat ground, 2 cm range noise, hole cells on the crater
  (none appeared anywhere else in any run):

  | Crater (width x depth) | 1.5 m | 2.0 m | 2.5 m | 3.0 m | 3.5 m | 4.0 m |
  | ---------------------- | ----- | ----- | ----- | ----- | ----- | ----- |
  | 0.50 x 0.45 m steep    | 24    | 17    | 14    | 12    | 6     | 0     |
  | 0.45 x 0.30 m bowl     | 13    | 13    | 10    | 4     | 0     | 0     |
  | 0.40 x 0.15 m bowl     | 10    | 6     | 3     | 0     | 0     | 0     |

- **Slopes:** a 0.45 x 0.30 m crater at 2 m is found on slopes from -20 to +20 degrees,
  with no false cells on the bare slope.
- **The real nodes, end to end.** Both node executables were run with `navigation.yaml`
  against a scripted rover driving at 0.28 m/s straight at a crater, with scans and TF
  published at 10 Hz:

  | Crater              | Local costmap lethal from | Global costmap lethal from | Global held it to 0.75 m? |
  | ------------------- | ------------------------- | -------------------------- | ------------------------- |
  | 0.50 x 0.45 m steep | 3.5 m                     | 3.4 m                      | yes                       |
  | 0.40 x 0.15 m bowl  | 2.6 m                     | 2.0 m                      | yes                       |

  For these runs the global node's `max_height_above_ground_m` was raised from 0.05 to
  local's 0.2. At 0.05 that existing test flagged about 90 cells of the synthetic ground
  away from the crater, with the hole test on or off, which hid the result being measured.
  The hole layer itself had no cell off the crater in any run.

  The global map is later on the shallow crater because it only accepts blobs of four
  connected cells (`min_obstacle_cells: 4`), and that crater does not show four until about
  2 m. With `hole_log_odds_hit` at the rock value of 0.25 it was later still, 1.7 m.

- **Cost:** the hole test added about 0.5 ms to one local classification and 1.2 ms to one
  global one on a desktop, with about 40 000 points in the buffer. Not measured on the
  Orange Pi, and the real buffer may hold several times more points.

Not checked:

- **Gazebo.** The scans captured from the sim on 2026-10-08 were not kept, so the new C++
  has not been run on them. The earlier offline prototype was (notes, section 7).
- **Real lidar data.** The synthetic lidar has clean geometry and simple noise. The
  threshold in particular is not validated.
- **The rover itself in view.** The synthetic sensor does not see a bucket.

## In Gazebo (2026-10-13)

Cratered arena (`~/perseus_simulation`, five craters), both real nodes with
`navigation.yaml`, simulated MID-360 scan pattern, rover pose from Gazebo ground truth,
bucket lowered to lift 30 degrees. The rover drove a 7 m path with four turns that passed
all five craters.

| Crater (width x depth) | First marked as a hole at      | In the global costmap at the end |
| ---------------------- | ------------------------------ | -------------------------------- |
| 1: 0.50 x 0.45 m steep | 2.7 m (in view from the start) | yes, 52 cells                    |
| 2: 0.45 x 0.30 m bowl  | 3.2 m                          | yes, 17 cells                    |
| 3: 0.40 x 0.15 m bowl  | 2.3 m live (2.7 m in replay)   | yes, 39 cells                    |
| 4: 0.50 x 0.45 m steep | 3.0 m live (3.4 m in replay)   | yes, 25 cells                    |
| 5: 0.45 x 0.40 m bowl  | 2.5 m (in view from the start) | yes, 14 cells                    |

Hole cells away from every crater: none in the live run. Replaying the recorded scans
offline gave up to 4 such cells in 14 of 856 frames, 0.7 to 1 m ahead of the rover while
it was turning; not explained yet.

What this run does not show:

- With the bucket at its start-up pose, crater 1 dead ahead was not seen at all. The
  bucket hides the ground ahead; the detector cannot fix that.
- The sim ran at a tenth of real time on the laptop's integrated GPU so the lidar could
  keep its 10 Hz. The NVIDIA driver was unusable (kernel module 595.91, libraries 595.99;
  a reboot should fix it).
- The rover pose was exact. Real odometry drifts.
- The craters were in five fixed places, and the rover took one path.

## Known limits of this version

- **Sharp troughs count as holes.** Undulation of 16 cm peak to trough over a 1.5 m
  wavelength gave false cells; 10 cm over 3 m gave none. A dip 1.0-1.5 m wide is flagged
  at 20 cm deep and not at 12 cm. Whether a 20 cm dip should be avoided is a fair
  question; the test says yes.
- **Shallow craters are seen late**: a 15 cm deep one from about 2.5 m, and not until 2 m
  in the global map.
- **Narrow ruts are mostly invisible**, as predicted in the notes (4.7).
- **Seen from far away, a known crater's far-wall cells read as free ground** and lose
  score in the global map. The floor cells are unobserved and keep theirs. The crater is
  re-marked on approach. A rule to stop that erosion was planned (notes 5.4) and
  deliberately left out of this basic version.
- **Two existing tests in the global map are stricter than the local ones** and, on the
  synthetic data, flagged cells that had nothing to do with craters. That behaviour is
  the same with the hole test switched off.

## What to do next

1. Run it in Gazebo on the cratered arena and look at the six RViz layers above.
2. Record a bag of the real lidar driving at a real hole, replay it, and tune
   `hole_depth_m`.
3. Port it to the branch the rest of autonomy is on. The classifier files are identical
   there, so the diff should apply as it is; `navigation.yaml` will need the `hole_*`
   lines added by hand.
