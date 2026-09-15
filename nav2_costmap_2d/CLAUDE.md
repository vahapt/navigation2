# nav2_costmap_2d — local notes

This is an in-place modified copy of nav2_costmap_2d in the `nav2_ws` overlay
(`/home/vahapt/temporary_overlays/nav2_ws`). The kr_robot_ws sim loads
`install/nav2_costmap_2d/lib/libnav2_costmap_2d_core.so` from this overlay.

## Feature: velocity-biased rolling window (uncommitted, in progress)

Goal: with `rolling_window: true`, shift the window center along the robot velocity
in `global_frame`, so a 10x10 m costmap can see e.g. 8 m ahead / 2 m behind instead
of growing width/height/resolution.

### Design
- `LayeredCostmap` is the single source of truth for the rolling origin:
  `origin = robot - size/2 + offset`. New API in `layered_costmap.hpp`:
  `setRollingWindowOffset(dx, dy)` and `getRollingWindowOrigin(x, y)`.
  The requested (unsnapped) origin is stored, so layers run the same grid-snap
  math as the master and stay cell-aligned.
- `ObstacleLayer`, `VoxelLayer`, `RangeSensorLayer` call `getRollingWindowOrigin()`
  in `updateBounds()` instead of computing `robot - size/2` themselves.
- `StaticLayer` (rolling branch) reports bounds from the master origin/size, not `robot ± half`.
- `Costmap2DROS::updateMap()` computes the offset with the free function
  `computeRollingWindowOffset()` (`costmap_2d_ros.hpp/.cpp`): velocity = difference
  of `lookupTransform(global, base)` at the latest stamp and `averaging_interval`
  earlier, times `lookahead_time`, clamped per axis to `max_offset`, and further
  clamped to `size/2 - resolution` so the robot stays in the window. On TF failure the
  offset is zero, with a throttled warning.
- `tf2::BufferCore::lookupVelocity` was tried and rejected: on Jazzy it returned
  wrong-frame/sign velocities for every frame argument combination and rejects `TimePointZero`.

### Parameters (flat names — dotted names are skipped by the dynamic param callbacks)
All are dynamically settable.
```yaml
rolling_window_offset_enabled: false          # disabling resets the offset to zero
rolling_window_lookahead_time: 1.0            # s
rolling_window_max_offset_x: 0.0              # m
rolling_window_max_offset_y: 0.0              # m
rolling_window_velocity_averaging_interval: 0.2  # s
```
Commented example added to `nav2_bringup/params/nav2_params.yaml`.

### Files touched
`include/nav2_costmap_2d/{layered_costmap,costmap_2d_ros}.hpp`,
`src/{layered_costmap,costmap_2d_ros}.cpp`,
`plugins/{obstacle_layer,voxel_layer,range_sensor_layer,static_layer}.cpp`,
`test/integration/obstacle_tests.cpp` (`testRollingWindowOffset`),
`test/unit/rolling_window_offset_test.cpp` (new) + `test/unit/CMakeLists.txt`.
`git status` in `src/navigation2` shows the full diff.

### Build / test
```bash
cd /home/vahapt/temporary_overlays/nav2_ws
colcon build --symlink-install --packages-select nav2_costmap_2d \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
build/nav2_costmap_2d/test/unit/rolling_window_offset_test
build/nav2_costmap_2d/test/integration/obstacle_tests
```
- New tests pass; full suite 176 tests, only failure is `obstacle_layer_test`, where all
  13 cases pass but the process aborts at exit inside rmw_zenoh (not yet confirmed on a clean tree).
- Clean rebuilds (deleting build/ or install/) are Vahap's call.

### Sim verification (2026-09-14, kr_robot_ws `scripts/sim_up.sh d250 warehouse`)
- Env: `ROS_DOMAIN_ID=2 RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.
- Node: `/XXnamespaceXX/local_costmap/local_costmap`, `global_frame: map`, `base_footprint`,
  10x10 m at 0.10 m. Drive via `/XXnamespaceXX/cmd_vel_smoothed` (TwistStamped, passes
  collision_monitor); TF on `/XXnamespaceXX/tf`.
- Probe script (drive + log window center − robot): was in the session scratchpad as
  `offset_probe.py`; recreate if needed (OccupancyGrid origin from `local_costmap/costmap`
  + `map→base_footprint` TF).
- Results with lookahead 4 s, max offset 3 m:
  - stopped → ≤ 0.1 m (one cell)
  - +0.5 m/s → +1.7…1.9 m (expected 2.0; lag from 2 Hz publish)
  - −0.3 m/s → −0.9…−1.15 m (expected −1.2)
  - lookahead 20 s at 0.3 m/s → capped at 2.8…3.0 m
  - no local_costmap warnings/errors in the container log.
- Robot config `kr_robot_ws/ros2_src/kr_ros2_industrial/karinca_d250_v8/config/karinca_d250_v8_config.yaml`
  (`local_costmap`) now has the feature enabled with the test values above
  (4.0 s lookahead, 3.0 m max offsets, 0.2 s averaging). These were picked for the test, not tuned.

### Open items
- Measure the CPU gain: compare `Map update time` (debug log) for a smaller window with
  offset vs. the current 10x10 centered window.
- Full navigation runs: check controller behavior with only `size/2 - max_offset` behind
  the robot (reversing / rotating in place right after fast motion).
- Tune `lookahead_time` / `max_offset_*` for the D250.
- Possible follow-up: rate limit on offset changes if origin jitter shows up in profiling.
- Third-party layers that still compute `robot - size/2` would drift from the master while an
  offset is active; they should use `getRollingWindowOrigin()`.
