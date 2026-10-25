# Bucket behaviours

Bucket behaviours built from saved poses, edited in Groot2 and run with `bucket_cli`.

| File               | What it is                                                                  |
| ------------------ | --------------------------------------------------------------------------- |
| `bucket_poses.xml` | One `Pose_<name>` subtree per saved pose, written by `bucket_teach.py`      |
| `collect.xml`      | `collect`, the mission's dig: mission.xml's `DigBucket` runs it             |
| `dump.xml`         | `dump`, the mission's dump: mission.xml's `DumpBucket` runs it              |
| `example_dump.xml` | `ExampleDump`, the old mission dump built from poses, as a template         |
| `bucket.btproj`    | The Groot2 project: every XML file here, plus the custom nodes' port models |

## Workflow

1. **Start the bucket.** Use the rover (`perseus.launch.py payload:=bucket`), the bench
   (`payloads bucket.launch.py`), or a simulated bucket with no hardware. The simulated
   bucket runs the real stack against `rover_can_sim.py` on vcan0. Create vcan0 once per
   boot with `software/scripts/vcan-setup.sh`. Then:

   ```sh
   ros2 launch payloads bucket.launch.py sim:=true   # or the rover / bench
   ros2 run payloads bucket_teach.py
   ```

   With `sim:=true`, RViz opens showing the bucket, rams included, following every
   move. Use `rviz:=true` to open it on the bench too, or `rviz:=false` to keep it shut.

2. **Zero the encoders** with `z`. Pick a bank (`lift`, `tilt`, `jaw`); both of its
   encoders are zeroed. This sends `/bucket/zero_encoder`, which takes the bucket from
   autonomy and sends the zero frame. The bucket firmware doesn't handle that frame yet,
   so for now this only works on the simulated bucket.

3. **Teach poses**, repeating as needed:
   - Jog with `w`/`s` (lift), `a`/`d` (tilt) and `q`/`e` (jaw). `+`/`-` change the duty.
     If an axis moves the wrong way, start with `--invert`.
   - Press `p` to save the bucket's current angles as `Pose_<name>`. Choose which joints
     the pose sets; the others hold. Also set `timeout_s` and `attempts`.
   - Press `g` to send the bucket back to a saved pose and check it.

4. **Build the behaviour in Groot2.** Open `bucket.btproj`. Every `Pose_*` appears as a
   subtree: drag poses into a new tree and wire them with Sequence, Fallback,
   RetryUntilSuccessful, Sleep and so on. Save the new tree as its own `.xml` file in this
   directory.

5. **Run it and watch it.** Press `r` in the teach console, or run:

   ```sh
   ros2 run mission_bt_server bucket_cli run MyTree --rearm
   ```

   In Groot2, switch to Monitor and connect to the rover's address on port 1667. The tree
   lights up as it runs. `--rearm` hands the bucket back from the operator first.

   Groot2 can only connect while a tree is up. With `r`, the run waits 3 s before
   moving so you can click Connect, and keeps the finished tree on Groot2 for 30 s.
   Change these with `bucket_teach.py --groot-delay S --groot-hold S`; space ends the
   hold early. From `bucket_cli`, pass `--start-delay S` and `--hold S` yourself. Pressing
   space in the console, or Ctrl-C in `bucket_cli`, halts the tree, and the bucket holds
   where it is. Moving the gamepad or a jog key always takes the bucket back from the tree.

## How the moves behave

Every pose is a **direct** `MoveBucket`: the setpoint jumps to the pose, and the bucket
board drives there at full duty. That gives one smooth motion. The board's position mode
has no speed setting yet, so the speed can't be set per pose.

The other option is a timed move (`duration_s`). There the trajectory controller creeps
the setpoint, and the board chases it in full-duty bursts, which judders. Only the
mission's dig carry uses a timed move, because its lift has to keep pace with the drive.

A pose that doesn't arrive within `timeout_s` is sent again, up to `attempts` times. The
retries are only for a missed pose. An operator override, a missing controller or a
rejected goal fails at once.

## In the mission

mission_bt_server registers every `.xml` here alongside mission.xml each time a mission
starts, with the same blank-port handling as `bucket_cli`. So an edit to `collect` or
`dump`, or to a pose, takes effect on the next mission without a rebuild. A file here
that doesn't load makes the mission fail to start, and the error names the file.
`bucket_bt_dir` on mission_bt_server points it at another directory.

`bucket_cli dig` and `bucket_cli dump` run the same `collect` and `dump` trees, through
mission.xml's `DigBucket` and `DumpBucket`.

## Notes

- Poses read the action name from `{@bucket_action}` on the root blackboard. That means
  a pose works inside any tree, with or without `_autoremap`. Your own nested subtrees
  that use other blackboard entries still need `_autoremap` ticked in Groot2.
- `bucket_cli run` also loads mission.xml, so your trees can use its subtrees, such as
  `JawMove` and `PrepareBucket`. Groot2 only knows about the files in this directory.
- After adding a tree file outside Groot2, regenerate the project with
  `ros2 run mission_bt_server bucket_cli project`. The teach console does this after
  every saved pose.
- The tools write here, in the source tree, by following colcon's symlink-install links,
  so new poses and trees show up in `git status`. `--dir` points them somewhere else.
- `mission_bt_server` also listens on Groot2 port 1667. When both run on the same
  machine, give `bucket_cli` a different port with `--groot-port`.
