# service-navigation-rbnx

Robonix wrapper around a system-installed ROS 2 Nav2 stack. The service
discovers map, odometry, and lidar inputs through Atlas and exposes
`robonix/service/navigation/*` over gRPC and MCP.

## Deployment config

Navigation behavior belongs to the robot deployment, not this provider. Each
robot must carry a complete Nav2 YAML and reference it with `params_file`:

```yaml
service:
  - name: nav2
    url: https://github.com/syswonder/service-navigation-rbnx
    branch: main
    config:
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan: lidar
```

Relative paths are resolved from the directory containing
`robonix_manifest.yaml`. `config/nav2_params.example.yml` is a neutral example,
not a robot profile. Copy it into the deploy repository and set the robot's
frames, footprint, velocity and acceleration limits, costmap layers, goal
tolerances, topics, and planner/controller plugins there.

For a 3D lidar, bind `scan_cloud` and declare the adapter explicitly:

```yaml
      params_file: config/nav2_params.yaml
      provider_ids:
        map: mapping
        odom: chassis
        scan_cloud: lidar3d
      scan_projection:
        enabled: true
        target_frame: base_link
        min_height_m: 0.1
        max_height_m: 1.5
        range_max_m: 12.0
```

Optional `bt_xml_file` points to a deploy-owned BehaviorTree XML. Existing
`params_profile` deployments remain supported and emit a migration warning;
new deployments should not use that field. See `config.spec` for every
accepted instance field and default.

## Runtime

At `Driver(CMD_INIT)`, the wrapper:

1. resolves the selected Atlas providers;
2. resolves and materializes the deployment-owned Nav2 YAML;
3. starts an optional PointCloud2-to-LaserScan adapter;
4. starts Nav2 and waits for the `navigate_to_pose` action server;
5. exposes navigate, status, and cancel capabilities.

Missing required providers return `deferred`. Invalid config or a Nav2 startup
failure returns `error` and tears down every child process.

## Build and tests

Navigation generates only its Atlas MCP bindings on every deployment target.
It deliberately does not generate, build, or source a Robonix ROS 2 IDL
overlay: the provider talks to Nav2 through the ROS 2 interfaces supplied by
the selected Humble installation, while its own public capability transport is
gRPC/MCP. Jetson native builds source only the system ROS 2 installation and
the locally built terminal-controller plugin overlay.

```bash
bash scripts/build.sh
python3 -m unittest -v \
  test_configuration.py \
  test_runtime_integration.py \
  test_rotation_guard.py \
  test_scan_filter.py
```

Jetson native builds require ROS 2 Humble and Nav2 packages compatible with the
host JetPack image. Docker manifests remain available for simulator and CI
deployments.
