# `test/autonomy` PR split plan

Splits `test/autonomy` (525 commits, about 55k added lines, 22 ROS packages of which 9 are new, plus firmware, Nix and web UI) into staged PRs.

Every PR is opened from `main`. PRs in the same stage are independent of each other. A stage opens only after every PR in the previous stage has merged.

The dependencies below come from each `package.xml` plus a grep for launch-file and share-directory lookups. Re-check them when cutting each PR.

## Ground rules

- **Cutting a PR:** branch from `origin/main`, then `git checkout test/autonomy -- <paths>`. Do not cherry-pick; the 525 commits include WIP and fixups.
- **Shared files:** `nix/extra-packages/ros-packages/perseus/overlay.nix`, `interfaces/CMakeLists.txt`, `typos.toml` and `software/web_ui/src/lib/scripts/rosTypes.ts` are touched by several PRs. Each PR takes only its own hunks (`git checkout -p`).
- **Self-contained manifests:** a PR's `package.xml` may only name packages that already exist on `main` or are in that PR.
- **Nix files:** CI regenerates Nix packages from `package.xml`, so the matching `nix/extra-packages/ros-packages/perseus/*.nix` ships in the same PR.
- **Naming:** `feat/` or `bugfix/` branches (not `test/`), Conventional Commit messages, no Claude trailer.
- **Cleanups:** fix naming and function-length findings inside each PR, not beforehand.
- **End check:** after the last PR merges, `git diff origin/main test/autonomy` should be empty.

## Stage 1: needs only `main`

All of these can open at once.

| PR                                   | Contents                                                                                                                                                                                                | Notes                                                                                    |
| ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `interfaces`                         | 8 new msgs, 4 srvs, edit to `MobilityStatus`                                                                                                                                                            | Nearly everything later depends on it.                                                   |
| `hi-can`                             | `hi_can_address.hpp`, `hi_can_parameter.hpp`, generator                                                                                                                                                 | Needed by payloads and firmware.                                                         |
| Dev tooling                          | `devenv.*`, profiles (`cyclonedds`, `hostnames`, `dev`, `prod`), `treefmt`, `typos`, colcon overlay, `local-date`, `config/*.toml`, `roar-launch.py`, `sync-time.sh`, `poweroff-rover.sh`, `.gitignore` | Carries the shellcheck `SC2016` fix. Check the profiles do not reference later packages. |
| Nix third-party                      | `orb-slam3` (+ patches), `bievr-lio*`, `groot2`, removal of `fast-lio`                                                                                                                                  | Heavy CI build. Check nothing else uses `fast-lio`.                                      |
| `can_diag` + `can_logger`            | 2 new packages                                                                                                                                                                                          | No local dependencies.                                                                   |
| `sensors`                            | `cloud_mesher`, `voxel_downsampler`, `height_crop`, launch files                                                                                                                                        |                                                                                          |
| `teleop`                             | controller and console changes                                                                                                                                                                          |                                                                                          |
| `description`                        | bucket URDF and meshes, `bucket_ram_follower.py`                                                                                                                                                        |                                                                                          |
| `hardware` + `footprint_broadcaster` | +46 and +1 lines                                                                                                                                                                                        | Small, so combined.                                                                      |
| `local_traversability`               | new package                                                                                                                                                                                             |                                                                                          |
| Firmware shared libs                 | `firmware/shared/modbus-core`, `firmware/shared/rs485`, `firmware/components/*` wrappers                                                                                                                | No ROS involved.                                                                         |

## Stage 2: needs Stage 1

| PR                           | Needs                                                                                            |
| ---------------------------- | ------------------------------------------------------------------------------------------------ |
| `system_monitor`             | `interfaces`                                                                                     |
| `link_monitor`               | `interfaces`                                                                                     |
| `health_check`               | `interfaces`                                                                                     |
| `watchdog`                   | `interfaces`                                                                                     |
| `arena_server`               | `interfaces`                                                                                     |
| `global_traversability`      | `local_traversability`                                                                           |
| `vision`                     | `interfaces`, `sensors`, `orb-slam3`                                                             |
| `payloads`                   | `interfaces`, `description`, `hi-can`                                                            |
| Firmware `encoder-board`     | firmware shared libs                                                                             |
| Firmware `excavation-bucket` | firmware shared libs, `hi-can`                                                                   |
| Web UI bucket widgets        | nothing at build time; needs the payloads topics at runtime, so it can go in any stage from 1 on |

## Stage 3: needs Stage 2

| PR                            | Needs                                                  |
| ----------------------------- | ------------------------------------------------------ |
| `rviz_plugins`                | `arena_server`, `interfaces`                           |
| `mission_bt_server`           | `interfaces`, `arena_server` at runtime, `groot2`      |
| `perseus` (launch and config) | `can_logger`, `hardware`, `payloads`, `system_monitor` |

## Stage 4: integration

| PR                 | Notes                                                                                                                                                                                 |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `autonomy_bringup` | The hub (+7961/-859). Its manifest lists most other packages, so it must be last. It could split into navigation and localisation launch/config, and behavior trees and mission data. |

## Decisions

1. **`payloads` and `mission_bt_server`.** `payloads` has an `exec_depend` on `mission_bt_server` (`bucket_teach.py` runs through `bucket_cli`), but `mission_bt_server` is in Stage 3. Plan: drop that line from the Stage 2 `payloads` PR and re-add it, with the `payloads.nix` change, in the `mission_bt_server` PR. The alternative is moving `payloads` to Stage 3.
2. **`interfaces`.** Keep it as one PR. Splitting by consumer makes every sibling PR conflict in `CMakeLists.txt`.
3. **Sibling conflicts.** PRs in one stage conflict on one line each in `overlay.nix`. Merge them one at a time and rebase the rest.

## Not yet verified

- Whether the Stage 1 dev tooling PR references packages from later stages.
- Whether anything besides `bievr-lio` uses the `fast-lio` Nix package being removed.
- Runtime-only coupling (topic names, launch files) is a grep heuristic, not a test.
