---
description: Goal-based navigation using a deploy-configured ROS 2 Nav2 stack.
---

# Navigation capability

Provides:

- `robonix/service/navigation/driver`
- `robonix/service/navigation/navigate`
- `robonix/service/navigation/navigate/status`
- `robonix/service/navigation/navigate/cancel`
- `robonix/service/navigation/set_speed`

Consumes Atlas-selected providers for:

- `robonix/service/map/occupancy_grid`
- `robonix/primitive/chassis/odom`
- either `robonix/primitive/lidar/lidar` or
  `robonix/primitive/lidar/lidar3d`

The robot deployment must provide a complete Nav2 YAML through `params_file`.
Relative paths use the directory containing `robonix_manifest.yaml`. A 3D lidar
also requires explicit `scan_projection` config. Optional `bt_xml_file` selects
a deploy-owned BehaviorTree. See `config.spec` for the complete instance config.

`set_speed` changes Nav2's live controller limit without restarting the service
or replacing the active goal. `faster` and `slower` adjust the current
percentage; `set` applies an explicit percentage; `reset` restores the
deployment default. Percentages are always bounded by the robot's
deployment-owned Nav2 maximum.

The service does not own mapping, robot TF, body dimensions, hard motion
limits, or robot-specific planner/controller tuning.
