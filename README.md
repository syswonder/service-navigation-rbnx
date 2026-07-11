# nav2_wrapper_rbnx

Robonix service package that wraps a standard [Nav2](https://navigation.ros.org/)
stack and owns the `robonix/service/navigation/*` capability. It routes the
Robonix gRPC/MCP contracts (`navigate`, `navigate/status`, `navigate/cancel`) onto Nav2's
`navigate_to_pose` action, and discovers every topic it consumes (`/map`,
`/odom`, `/scan`) through atlas — so the same package drops onto any body that
publishes those contracts, in simulation or on real hardware.

It is body- and scene-agnostic: nothing in the package names a specific robot,
topic, or provider. The deploy manifest picks a params profile and a build
target; everything else is resolved at runtime.

---

## Capability surface

| Contract                              | Transport | Handler                                                 |
| ------------------------------------- | --------- | ------------------------------------------------------- |
| `robonix/service/navigation/driver`   | gRPC      | `Driver(CMD_INIT/SHUTDOWN, …)` — lifecycle (Service)    |
| `robonix/service/navigation/navigate` | gRPC + MCP | `Navigate(PoseStamped)` → dispatches a Nav2 action goal |
| `robonix/service/navigation/navigate/status` | gRPC + MCP | `GetNavigationStatus(run_id)` → status from cache |
| `robonix/service/navigation/navigate/cancel` | gRPC + MCP | `CancelNavigation(run_id)` → Nav2 `cancel_goal_async` |

The driver + the three data contracts are hosted and declared on atlas by
`robonix_api.Service.run()`. `navigate` / `status`
/ `cancel` are live from process start; each guards on the rclpy node, so a call
that lands before `CMD_INIT` finishes returns a clean "not initialized" instead
of crashing.

## Lifecycle (how a deploy brings nav2 up)

The package is a `robonix_api.Service`. `scripts/start.sh` launches the bridge
(in a container or natively — see *Deployment targets*); the bridge registers
`nav2` with atlas, serves the Driver gRPC, and blocks awaiting
`Driver(CMD_INIT, config_json)` from `rbnx boot`. There is no config on disk or
in the environment — the config dict arrives only over this gRPC channel.

On `CMD_INIT` the `on_init` handler:

1. Resolves upstream deps from atlas **by contract** (`ATLAS.find_capability`),
   never by hardcoded topic name. If a **required** dep is missing it returns
   `Deferred(...)` — rbnx retries once the upstream provider registers, so nav2
   is never spawned half-wired.
2. Spawns `ros2 launch nav2_bringup navigation_launch.py` with the selected
   params profile, `use_sim_time`, and the resolved topics passed as launch
   remaps.
3. Brings up an rclpy node + `navigate_to_pose` ActionClient and waits
   (`action_wait_s`) for the Nav2 lifecycle to advertise the action server.

`CMD_SHUTDOWN` (or a process signal) tears the Nav2 subprocess down via
`on_shutdown` so it never outlives the wrapper.

## Quick start (Webots simulation)

A minimal deploy that proves nav2 navigates the Webots Tiago lives in the
robonix repo at `examples/webots/nav2_test.yaml` (atlas + executor + chassis +
lidar + mapping + nav2). With the Webots sim already running:

```bash
# 1. one-time: build the nav2 docker image (x86 + docker is the default target)
rbnx build -p /path/to/nav2_wrapper_rbnx        # builds image `robonix-nav2`

# 2. boot the stack
cd <robonix>/examples/webots
rbnx boot -f nav2_test.yaml                      # nav2 reaches ACTIVE

# 3. send a goal over the navigate gRPC and watch /cmd_vel move the robot
#    (goal in the map frame for the sim profile; see Params profiles)
```

The `sim` params profile navigates in the **map frame with online SLAM**: an
rtabmap mapping provider publishes both a live `/map` and a `map→odom` TF. Both
costmaps anchor in `map` and a `StaticLayer` tracks `/map` as it grows
(`map_subscribe_transient_local`), so the robot maps and navigates at the same
time — obstacles populate the costmaps straight from the SLAM grid, and the
`ObstacleLayer` adds anything not yet mapped. It needs a SLAM provider that
owns `map→odom` (rtabmap in the Webots example); no AMCL or pre-built map.

## Deployment targets

Nav2 itself is large and the upstream `apt` packages build cleanly on Humble, so
this package vendors YAML plus one small Ranger terminal plugin. Three targets cover the
common deployments; the target is selected by `RBNX_BUILD_TARGET` (build) and the
matching per-target package manifest (`rbnx boot -f … --manifest`):

| `RBNX_BUILD_TARGET` | package manifest                  | Where Nav2 runs                                     |
| ------------------- | --------------------------------- | --------------------------------------------------- |
| `x86-docker` (default) | `package_manifest.yaml`        | x86_64 host, Nav2 in the `robonix-nav2` image       |
| `jetson-docker`     | `package_manifest.jetson-docker.yaml` | arm64 Jetson, same Dockerfile (ros:humble is multi-arch) |
| `jetson-native`     | `package_manifest.jetson-native.yaml` | arm64 Jetson with host ROS2 — no docker        |

- **Docker targets** (`x86-docker`, `jetson-docker`) build the `robonix-nav2`
  image from `docker/Dockerfile` (ROS2 Humble + `navigation2` + `nav2-bringup` +
  `pointcloud-to-laserscan`). `scripts/start.sh` runs it with `--network host
  --ipc=host`, bind-mounting the package and `robonix-api`. Use this where the
  host has no ROS2.
- **Native target** (`jetson-native`) requires `ros-humble-nav2-bringup` +
  `ros-humble-navigation2` on the host:

  ```bash
  sudo apt install ros-humble-nav2-bringup ros-humble-navigation2 \
                   ros-humble-pointcloud-to-laserscan ros-humble-rtabmap-util
  ```

  `scripts/build.sh` verifies the host packages; `scripts/start_native.sh`
  sources the host ROS2 and runs the bridge directly. The dispatch honours
  `ROBONIX_NAV2_FORCE` / `ROBONIX_NAV2_PLATFORM` overrides.

To add a target, drop a new `package_manifest.<target>.yaml` whose `build:` /
`start:` lines set `RBNX_BUILD_TARGET` and run the right scripts; the dispatch in
`build.sh` / `start.sh` already keys off that variable.

## Config (passed via `Driver(CMD_INIT, config_json)`)

```yaml
- name: nav2
  url: https://github.com/syswonder/service-navigation-rbnx
  branch: main
  config:
    params_profile: sim       # sim | slam | default  → config/nav2_params_<profile>.yml
    params_file: ""           # absolute / package-relative path to override entirely
    use_sim_time: true        # true under a simulator that publishes /clock
    action_wait_s: 90.0       # how long to wait for navigate_to_pose to advertise
    topic_remap: {}           # per-key override of atlas-resolved bindings
```

`params_profile` selects `config/nav2_params_<profile>.yml`. `params_file`
(absolute, or relative to the package root) overrides it entirely — useful when
an operator's tuning lives outside the package.

### Params profiles

| Profile   | Global costmap frame | Localization      | Use when                                          |
| --------- | -------------------- | ----------------- | ------------------------------------------------- |
| `sim`     | `map` (static layer) | online SLAM (rtabmap) | Webots / any body with a SLAM provider publishing `/map` + `map→odom`, mapping while navigating |
| `slam`    | `map`                | AMCL + map        | Real deploy with a SLAM `map→odom` TF + static map |
| `default` | `map`                | AMCL + map        | Generic map-based navigation                       |
| `ranger_mini_v3` | `map`          | external SLAM     | Ranger Mini v3 with bounded, replan-persistent terminal rotation |

The Ranger profile keeps DWB for path tracking but replaces Humble's stock
terminal state. `ControllerServer` resets the goal checker and DWB critics each
time a replanned path arrives; at 5 Hz this clears the normal `RotateToGoal`
XY latch and can re-enable translation after skid-steer rotation drifts XY.
`PersistentGoalChecker` and `PersistentRotateToGoalCritic` reset only when the
actual goal changes. Once the robot enters 0.30 m, translation stays disabled;
0.45 m XY drift, 15 seconds, three seconds without yaw progress, or excessive
cumulative rotation stops the controller with an actionable failure. Terminal
angular velocity is capped at 0.30 rad/s. The Ranger recovery tree has no spin
or backup motion.

Daytime acceptance uses `scripts/ranger_acceptance.py PLAN.yaml`. It is a dry
run by default; sending goals requires both `--execute` and
`--i-understand-robot-will-move`. Each goal is sent once with no automatic
retry. The JSON report records terminal XY drift, accumulated rotation,
commands, stopped velocity, duration, and the Nav2 action result.

## Atlas contract dependencies

The wrapper resolves every topic it consumes through atlas
(`ATLAS.find_capability` + `connect_capability`) — it does NOT know which package
provides them on a given deploy. Each resolved topic is passed to `nav2_bringup`
as a launch remap.

**Required** (`on_init` returns `Deferred` until both are on atlas):

| Contract                              | Resolved into nav2 as |
| ------------------------------------- | --------------------- |
| `robonix/service/map/occupancy_grid`  | `/map`                |
| `robonix/primitive/chassis/odom`      | `/odom`               |

**Optional** (init proceeds if absent; that observation source is just off):

| Contract                          | Resolved into nav2 as |
| --------------------------------- | --------------------- |
| `robonix/primitive/lidar/lidar`   | `/scan`               |
| `robonix/primitive/lidar/lidar3d` | `/scanner/cloud`      |

Override any binding via the manifest's `topic_remap` block (e.g. to pin one of
several providers, or tap a downstream filter):

```yaml
config:
  topic_remap:
    map: /robonix/map/occupancy_grid
    scan: /scanner_normalized
```

## TF prerequisites

Nav2 needs a healthy `odom → base_link → sensor` TF chain plus `map → odom` for
all three profiles (`sim` included — it navigates in the map frame). TF is still
a global ROS side-channel, not an atlas contract. The convention: the SLAM
provider owns `map → odom`, the chassis provider owns `odom → base_link`, and a
body-description provider owns `base_link → sensor_*` via `robot_state_publisher`.
Under `sim`, an online-SLAM provider (rtabmap in the Webots example) supplies
both `map → odom` and the `/map` the costmaps' static layer consumes.

If your costmap references `/scan` but only `primitive/lidar/lidar3d`
(PointCloud2) is on atlas, the Ranger profile owns the adapter. Set
`scan_deskewing: true` only when the cloud has per-point timestamps; it runs
RTAB-Map's lidar deskewing against the chassis odom TF before projection.

## DDS / sysctl tuning (multi-container deploys)

The docker targets run Nav2 with `--network host` and a UDP-only FastDDS profile
(`docker/no_shm_profile.xml`) — SHM is disabled because cross-container SHM
locks are unreliable, and the transport stays on all interfaces so the sim's
topics remain discoverable.

On a host with many DDS participants (a simulator brings up dozens), the default
`net.core.rmem_max` (208 KB) can overflow under the endpoint-discovery burst and
drop Nav2's lifecycle `change_state` replies, stalling bring-up
("failed to send response … timeout"). If you see Nav2 nodes stuck
`unconfigured` / `inactive`, raise the host UDP buffers — the same fix used for
high-rate lidar:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
# persist: echo 'net.core.rmem_max=2147483647' | sudo tee /etc/sysctl.d/10-dds.conf
```

On a robot with a quiet network and tuned buffers this is unnecessary.

## Layout

```
nav2_wrapper_rbnx/
├── package_manifest.yaml                 x86-docker (default)
├── package_manifest.jetson-docker.yaml
├── package_manifest.jetson-native.yaml
├── nav2_wrapper/
│   └── atlas_bridge.py                    Service: lifecycle + navigate/status/cancel gRPC
├── scripts/
│   ├── build.sh                           codegen + per-target build (RBNX_BUILD_TARGET)
│   ├── start.sh                           docker ↔ native dispatch
│   └── start_native.sh                    host ROS2 path
├── terminal_controller/                   persistent goal checker + DWB critic
├── docker/
│   ├── Dockerfile                         ROS2 Humble + Nav2 + pointcloud_to_laserscan
│   ├── entrypoint.sh
│   └── no_shm_profile.xml                 UDP-only FastDDS profile
└── config/                                vendored Nav2 params
    ├── nav2_params.yml
    ├── nav2_params_slam.yml
    └── nav2_params_sim.yml
```

## License

This package: Apache-2.0 (matches Nav2 upstream).
