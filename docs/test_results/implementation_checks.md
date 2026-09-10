# Implementation checks — 2026-09-08

Scope: ZED + RTAB-Map integration for the laptop bedroom trial. This is a software
validation record, not a completed bedroom mapping result.

## Environment

- Ubuntu-22.04 WSL environment on the laptop; source workspace mounted from Windows.
- ROS 2 Humble, Python 3.10.12, pytest 6.2.5.
- Installed RTAB-Map/rtabmap_slam 0.23.7 through Ubuntu/ROS packages for this check.
- `nvidia-smi -L` exposed an NVIDIA GeForce RTX 4050 Laptop GPU.
- ZED SDK headers were present. ZED wrapper source identifies itself as 5.2.2.
- No `/dev/video*` camera device was exposed and no SVO recording was available.
- The separate ZED wrapper checkout was not built in the isolated validation prefix.
- Changes remain uncommitted on top of the existing working tree; no map or Area
  Memory file was deleted or overwritten.

## Results

| Check | Result |
| --- | --- |
| Isolated `vision_bringup` CMake/C++ build | Pass |
| Launch, TF ownership, live/replay clocks and database mode contracts | Pass |
| Database preservation, invalid path/mode rejection, new-map handling | Pass |
| Installed launch/config/RViz assets | Pass |
| RViz assembled-cloud selection without display history | Pass (configuration check; GUI not exercised) |
| Actual RTAB-Map startup and parameter query | Pass |
| Actual RTAB-Map clean shutdown and SQLite database creation | Pass (no sensor observations) |
| Full camera launch and input synchronization | Not run |
| Bedroom accumulation, loop closure, saved geometry reload, relocalization | Not run |
| SVO regression / Orin | Not run |

Pytest discovered and passed **17 tests**, with no skipped tests. Colcon reports
18 entries because its summary includes the enclosing CTest entry. Final suite
runtime was about 2.25 seconds inside pytest.

The real-node test runs RTAB-Map on ROS domain 219, localhost only, with a
disposable database. It checks ROS parameter acceptance and confirms database
creation after clean shutdown; it makes no claims about reconstruction quality.

The mounted filesystem produced small Make clock-skew warnings during build.
Build completed successfully and the installed assets/tests passed. For subsequent
full wrapper builds, the Linux filesystem is preferable if mounted-path timestamp
warnings recur. This check did not rebuild or test the ZED SDK/wrapper binaries.

## Commands and artifacts

Commands used from `ros_ws/`, after sourcing `/opt/ros/humble/setup.bash`:

```bash
colcon --log-base log/codex_rtabmap build \
  --base-paths src/vision_bringup \
  --build-base build/codex_rtabmap --install-base install/codex_rtabmap \
  --symlink-install
source install/codex_rtabmap/setup.bash
colcon --log-base log/codex_rtabmap test \
  --base-paths src/vision_bringup \
  --build-base build/codex_rtabmap --install-base install/codex_rtabmap \
  --event-handlers console_direct+
colcon test-result --test-result-base build/codex_rtabmap --verbose
ros2 launch vision_bringup zed_rtabmap.launch.py --show-args
```

The restricted `--base-paths` builds this package independently of the separate,
unbuilt wrapper checkout. It is not a replacement for the full camera workspace
build documented in the README.

Local ignored artifacts:

- `ros_ws/build/codex_rtabmap/vision_bringup/test_results/vision_bringup/test_bringup.xunit.xml`
- `ros_ws/build/codex_rtabmap/vision_bringup/ament_cmake_pytest/test_bringup.txt`
- `ros_ws/log/codex_rtabmap/`

## Corrections established by testing

RTAB-Map 0.23.7's core subscriber does not declare `approx_sync_max_interval`.
The initial draft included that setting; the real-node test exposed it. The
delivered profile uses exact synchronization and explicitly preserves ZED
acquisition timestamps. Actual timestamp alignment still needs camera validation.

Unnecessary ground-segmentation overrides produced RTAB-Map warnings and were
removed. The delivered cloud uses the normal 3D map generation with voxel filtering.
The source-derived topic remappings and camera-root TF contract are checked by
the launch tests; real topic publication remains a hardware acceptance item.
