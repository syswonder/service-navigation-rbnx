# service-navigation-rbnx provider instance configuration
#
# This file documents the YAML object passed through a robot deployment entry.
# It is a human- and model-readable contract, not a separately parsed schema.

required:
  params_file:
    type: path
    path_base: directory containing robonix_manifest.yaml
    description: Complete deploy-owned Nav2 parameter YAML.
    example: config/nav2_params.yaml

  provider_ids:
    type: mapping[string, string]
    description: Atlas provider IDs for Navigation inputs.
    supported_roles:
      map: robonix/service/map/occupancy_grid
      odom: robonix/primitive/chassis/odom
      scan: robonix/primitive/lidar/lidar
      scan_cloud: robonix/primitive/lidar/lidar3d

optional:
  bt_xml_file:
    type: path
    path_base: directory containing robonix_manifest.yaml
    description: >-
      Custom BehaviorTree XML used when params_file contains the
      __ROBONIX_BT_XML__ token.

  use_sim_time:
    type: boolean
    default: false
  action_wait_s:
    type: number
    default: 45.0

  scan_projection:
    type: mapping
    description: >-
      Explicit PointCloud2-to-LaserScan adapter. Omit for a native LaserScan.
    fields:
      enabled: {type: boolean, default: false}
      target_frame: {type: string, default: Soma base_frame}
      min_height_m: {type: number, default: 0.0}
      max_height_m: {type: number, default: 2.0}
      range_max_m: {type: number, default: 30.0}
      self_filter_margin_m: {type: number, default: 0.05}
      transform_tolerance_s: {type: number, default: 0.15}
      deskewing: {type: boolean, default: false}
      deskew_fixed_frame: {type: string, default: odom}
      deskew_wait_for_transform_s: {type: number, default: 0.2}

  topic_remap:
    type: mapping[string, string]
    description: Advanced direct ROS topic override; provider_ids is preferred.
  trajectory_log_dir:
    type: path
    default: rbnx-build/data/trajectories
  guard_terminal_xy_m: {type: number, default: 0.45}
  guard_terminal_timeout_s: {type: number, default: 15.0}
  guard_no_progress_s: {type: number, default: 3.0}
  guard_global_spin_timeout_s: {type: number, default: 25.0}
  guard_global_spin_limit_rad: {type: number, default: 6.783185307}

deprecated_compatibility:
  params_profile:
    replacement: params_file
    behavior: Known legacy profiles still load and emit a migration warning.
  scan_deskewing:
    replacement: scan_projection.deskewing
  scan_self_filter_margin_m:
    replacement: scan_projection.self_filter_margin_m
  odom_frame:
    replacement: scan_projection.deskew_fixed_frame
