# nav2_wrapper_rbnx — capability surface

Wraps a system-installed ROS2 **Nav2** stack as a Robonix navigation service.
It registers the `robonix/service/navigation/*` contracts, discovers its map +
odom inputs via atlas, and brings up `nav2_bringup navigation_launch.py` as a
subprocess.

## Provides (declared on atlas)

| contract | transport | notes |
|---|---|---|
| `robonix/service/navigation/driver` | grpc | Driver lifecycle (INIT/SHUTDOWN) |
| `robonix/service/navigation/navigate` | grpc | navigate-to-pose action entry |
| `robonix/service/navigation/status` | grpc | current navigation status |
| `robonix/service/navigation/cancel` | grpc | cancel the active goal |

The three RPCs are backend-agnostic — identical to `simple_nav_rbnx`, so the
two are interchangeable from a consumer's view.

## Consumes (via atlas discovery)

Resolved at `Driver(CMD_INIT)`; override any binding with `cfg["topic_remap"]`.

| key | contract | role |
|---|---|---|
| `map` *(required)* | `robonix/service/map/occupancy_grid` | global costmap StaticLayer (mapping_rbnx) |
| `odom` *(required)* | `robonix/primitive/chassis/odom` | controller + AMCL odom |
| `scan` *(optional)* | `robonix/primitive/lidar/lidar` | 2D ObstacleLayer |
| `scan_cloud` *(optional)* | `robonix/primitive/lidar/lidar3d` | 3D VoxelLayer |

If a required dep is missing, INIT returns `deferred` (nav2 is not spawned)
rather than coming up half-wired.

> **Known limitation (tracked):** the velocity-command **output** (`cmd_vel`)
> is not yet atlas-discovered — nav2 publishes to the global `/cmd_vel`, so the
> chassis must listen there. And the resolved input topics are not yet rewritten
> into the Nav2 params at launch (they are passed as launch args that
> `navigation_launch.py` ignores). Both require a live Nav2 stack to implement
> and verify; until then, ensure your chassis subscribes to `/cmd_vel` and that
> the params profile's topic names match your robot. See README.

## Config (`config:` block → `Driver(CMD_INIT, config_json)`)

```yaml
config:
  params_profile: slam        # slam | sim | default  → config/nav2_params_<p>.yml
  params_file: ""             # absolute/pkg-relative override (wins over profile)
  use_sim_time: false
  action_wait_s: 45.0         # bring-up timeout for navigate_to_pose
  topic_remap: {}             # per-key override, e.g. { map: /my_map }
```

## Params profiles

| profile | file | use |
|---|---|---|
| `slam` *(default)* | `config/nav2_params_slam.yml` | map frame from external SLAM (mapping_rbnx); no map_server |
| `sim` | `config/nav2_params_sim.yml` | webots-tuned (TB3-like limits) |
| `default` | `config/nav2_params.yml` | static-map deploys |

> The robot footprint, velocity/accel limits, and frames currently live in
> these YAML files (Ranger-Mini / TB3 values). A different body must supply its
> own `params_file`. Making these config knobs (or sourcing them from `soma`'s
> body description) is tracked work — see README.

## What this does NOT do (deliberately)

- **No mapping / SLAM** — consumes `robonix/service/map/*` from `mapping_rbnx`.
- **No TF publishing** — the deploy must provide `map→odom→base_link→sensor`.
- **No pointcloud→laserscan conversion** — if the costmap wants `LaserScan`
  and you only have a 3D cloud, add a `pointcloud_to_laserscan` adapter.
