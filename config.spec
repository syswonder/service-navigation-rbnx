# service-navigation-rbnx deployment config schema.
#
# rbnx delivers the deployment entry's `config:` mapping through
# Driver(CMD_INIT). The wrapper validates it there; an invalid value returns
# an error and stops every child process it started, and a missing map or
# odom provider returns deferred.
#
# Relative file paths (params_file, bt_xml_file) resolve against the
# directory containing robonix_manifest.yaml (RBNX_INVOCATION_CWD; the
# package root when that variable is unset).
#
# Topic fields marked "absolute ROS topic name" must be "/" followed by
# "/"-separated tokens of letters, digits and underscores, with no token
# starting with a digit. Empty, relative, private (~), substituted ({...}),
# or trailing-slash names fail initialization.
specVersion: 1
description: >-
  Nav2 navigation service. The deployment supplies the Nav2 parameter file,
  the Atlas providers for map, odometry and lidar, and the speed policy
  enforced by the final velocity guard.

properties:
  params_file:
    type: string
    x-group: Nav2 parameters
    description: >-
      Path to the complete Nav2 parameter YAML owned by the deployment,
      absolute or relative to the directory containing
      robonix_manifest.yaml. The file must exist. Required unless the
      deprecated params_profile is set; when both are set, params_file wins.
      If the file contains any __ROBONIX_*__ token, the wrapper writes a copy
      with __ROBONIX_MAP_TOPIC__, __ROBONIX_ODOM_TOPIC__,
      __ROBONIX_SCAN_TOPIC__, __ROBONIX_SCAN_CLOUD_TOPIC__ replaced by the
      resolved input topics, __ROBONIX_FOOTPRINT__ by Soma's footprint
      polygon, and __ROBONIX_BT_XML__ by bt_xml_file. A token that cannot be
      resolved, or any other __ROBONIX_*__ token, fails initialization.

  bt_xml_file:
    type: string
    x-group: Nav2 parameters
    description: >-
      Path to a BehaviorTree XML owned by the deployment, absolute or relative
      to the directory containing robonix_manifest.yaml. It replaces the
      __ROBONIX_BT_XML__ token in params_file; without that token it has no
      effect. When set, the file must exist whenever params_file contains any
      __ROBONIX_*__ token. If params_file contains __ROBONIX_BT_XML__ and this
      key is absent, initialization fails.

  use_sim_time:
    type: boolean
    default: false
    x-group: Nav2 parameters
    description: >-
      Run Nav2 and the wrapper's ROS node on /clock simulated time. Enable it
      only when the whole TF and sensor graph publishes simulated time. The
      velocity guard always uses system time.

  provider_ids:
    type: object
    x-group: Inputs
    description: >-
      Atlas provider id (the deployment entry name) to use for each input
      role. A role that is absent or empty uses the first provider Atlas
      returns for its contract. map and odom are required inputs: if no
      provider is found, initialization is deferred until one registers. scan
      and scan_cloud are optional and are skipped when not found. Nav2
      receives each resolved topic only through the matching
      __ROBONIX_*_TOPIC__ token in params_file.
    properties:
      map:
        type: string
        x-provider: robonix/service/map/occupancy_grid
        description: Provider of the occupancy grid map.
      odom:
        type: string
        x-provider: robonix/primitive/chassis/odom
        description: Provider of odometry.
      scan:
        type: string
        x-provider: robonix/primitive/lidar/lidar
        description: >-
          Provider of a 2D LaserScan. When this role resolves, scan_projection
          is not started.
      scan_cloud:
        type: string
        x-provider: robonix/primitive/lidar/lidar3d
        description: >-
          Provider of a 3D PointCloud2. Required by scan_projection.
      depth:
        type: string
        x-provider: robonix/primitive/camera/depth
        description: >-
          Camera whose depth image and intrinsics (camera/intrinsics from the
          same provider) become obstacles the lidar cannot see, such as table
          tops. Used only when params_file contains
          __ROBONIX_DEPTH_CLOUD_TOPIC__, which is replaced by the cloud's
          topic; see depth_obstacles.

  depth_obstacles:
    type: object
    x-group: Inputs
    description: >-
      How the depth camera's image becomes the PointCloud2 behind
      __ROBONIX_DEPTH_CLOUD_TOPIC__: every stride-th pixel, within the range,
      at most rate_hz times a second. The cloud is in the camera frame; the
      costmap layers that read it place it through TF and choose the
      obstacle heights.
    properties:
      stride:
        type: integer
        minimum: 1
        maximum: 16
        default: 4
        description: Pixel step in both directions.
      min_range_m:
        type: number
        minimum: 0
        default: 0.2
        description: Nearer depth readings are dropped, in metres.
      max_range_m:
        type: number
        minimum: 0.5
        default: 4.0
        description: Farther depth readings are dropped, in metres.
      rate_hz:
        type: number
        minimum: 0.5
        maximum: 30
        default: 10
        description: Clouds published per second at most.

  topic_remap:
    type: object
    x-group: Inputs
    description: >-
      Fixed ROS topic for an input role, bypassing Atlas discovery for that
      role. It takes priority over provider_ids for the same role. Values are
      not validated. An empty value leaves the role unbound, which defers
      initialization for map and odom. Prefer provider_ids.
    properties:
      map:
        type: string
        description: Occupancy grid map topic.
      odom:
        type: string
        description: Odometry topic.
      scan:
        type: string
        description: LaserScan topic.
      scan_cloud:
        type: string
        description: PointCloud2 topic.

  dynamic_speed:
    type: object
    x-group: Speed control
    description: >-
      Policy for runtime navigation speed changes, in SI units. The wrapper
      sends percentage limits as nav2_msgs/SpeedLimit to Nav2's Controller
      Server and the final velocity guard enforces the resulting planar
      linear-speed limit independently. Angular limits stay in the Nav2
      parameter file. Keys other than those listed fail initialization.
    required: [max_linear_speed_mps, default_percentage]
    properties:
      max_linear_speed_mps:
        type: number
        description: >-
          Hard ceiling, in m/s, for planar linear speed sqrt(vx^2 + vy^2).
          Must be finite and greater than 0. Set it to the effective maximum
          of the selected Nav2 controller: max_speed_xy, or a stricter
          per-axis max_vel_x/max_vel_y. For a non-holonomic DWB setup with
          max_vel_x 0.3, max_vel_y 0 and max_speed_xy 0.3, use 0.3. The final
          guard never publishes a planar command above this value, including
          commands from Nav2 recovery behaviors.
      default_percentage:
        type: number
        maximum: 100
        description: >-
          Percentage of max_linear_speed_mps applied at startup and by the
          normal command; it is also the initial session limit. A goal-scoped
          change restores the session limit when the goal ends. Must satisfy
          min_percentage <= default_percentage <= 100. Example: 0.3 m/s at 75%
          gives 0.225 m/s.
      step_percentage:
        type: number
        default: 20
        maximum: 100
        description: >-
          Percentage points added by faster or removed by slower; 75% plus a
          20-point step is 95%. Must be finite and greater than 0. faster
          stops at 100.
      min_percentage:
        type: number
        default: 20
        maximum: 100
        description: >-
          Lowest percentage. slower stops here and set_speed_limit rejects
          lower values. Must be greater than 0 and at most default_percentage,
          so a default_percentage below 20 requires setting this key. It is
          not a stop command; use cancel or the chassis stop capability.
      topic:
        type: string
        default: /speed_limit
        description: >-
          Absolute ROS topic name carrying nav2_msgs/SpeedLimit; must equal
          the Controller Server's speed_limit_topic. Initialization fails if
          Nav2 does not subscribe within min(action_wait_s, 10) seconds.

  scan_projection:
    type: object
    x-group: Scan projection
    description: >-
      PointCloud2-to-LaserScan adapter for a 3D lidar. It publishes the
      filtered scan on /scanner/scan and binds it as the scan role. Omit it
      for a native LaserScan. When present, it is validated even if disabled,
      and keys other than those listed fail initialization.
    properties:
      enabled:
        type: boolean
        default: false
        description: >-
          Start the adapter. It runs only when no scan provider resolved; it
          then requires a resolved scan_cloud input, Soma's footprint
          capability and the pointcloud_to_laserscan package.
      target_frame:
        type: string
        description: >-
          TF frame in which point height, range and self-filtering are
          evaluated. Absent or empty uses Soma's base_frame.
      min_height_m:
        type: number
        default: 0.0
        description: >-
          Lowest point height kept, in metres, in target_frame. Must be less
          than max_height_m.
      max_height_m:
        type: number
        default: 2.0
        description: >-
          Highest point height kept, in metres, in target_frame. Must be
          greater than min_height_m.
      range_max_m:
        type: number
        default: 30.0
        minimum: 0
        description: Maximum projected range, in metres.
      self_filter_margin_m:
        type: number
        default: 0.05
        minimum: 0
        description: >-
          Margin, in metres, added to the circumscribed radius of Soma's
          footprint to form the scan's minimum range; closer points are
          treated as the robot itself and dropped.
      transform_tolerance_s:
        type: number
        default: 0.15
        minimum: 0
        description: TF timestamp tolerance for projection, in seconds.
      deskewing:
        type: boolean
        default: false
        description: >-
          Correct motion distortion with rtabmap_util lidar_deskewing before
          projection. Enable it only when the cloud has per-point timestamps
          and odometry TF covers the scan interval.
      deskew_fixed_frame:
        type: string
        default: odom
        description: >-
          Fixed TF frame for deskewing; empty also means odom. Used only when
          deskewing is true.
      deskew_wait_for_transform_s:
        type: number
        default: 0.2
        minimum: 0
        description: >-
          Maximum wait, in seconds, for the transforms deskewing needs. Used
          only when deskewing is true.

  velocity_output_topic:
    type: string
    x-group: Velocity output
    description: >-
      Absolute ROS topic name on which the final velocity guard publishes.
      When absent, the ROBONIX_VELOCITY_OUTPUT_TOPIC environment variable is
      used if set, otherwise /cmd_vel; this key takes priority over the
      environment. Use /robonix/nomotion/cmd_vel to keep a physical robot
      still during integration. An invalid name, including an empty one,
      fails initialization before any provider is resolved.

  controller_velocity_output_topic:
    type: string
    x-group: Velocity output
    description: >-
      Absolute ROS topic name for the raw velocity output of Nav2's
      controller_server and behavior_server. When absent, the controller
      publishes to /cmd_vel_nav and the behavior server feeds the guard
      directly. When set, a mux owned by the deployment must be the only
      publisher on /cmd_vel_nav; velocity_smoother still reads /cmd_vel_nav
      and the guard and velocity_output_topic are unchanged. If the key is
      present, an invalid value, including empty or null, fails
      initialization.

  action_wait_s:
    type: number
    default: 45.0
    x-group: Startup
    description: >-
      Maximum time, in seconds, initialization waits for Nav2's
      navigate_to_pose action server. On expiry initialization fails and the
      spawned Nav2, scan and guard processes are stopped. Must be greater
      than 0. It also caps the speed-limit subscription wait at
      min(action_wait_s, 10) seconds.

  # The guard counts the robot as rotating in place while the commanded
  # planar speed is at most 0.05 m/s and the commanded yaw rate is at least
  # 0.05 rad/s. When a limit trips, the guard requests cancellation of the
  # active NavigateToPose goal and publishes zero velocity until that goal
  # ends. The wrapper does not range-check these values; the minimums below
  # state the meaningful range.
  guard_terminal_xy_m:
    type: number
    default: 0.45
    minimum: 0
    x-group: Guard
    description: >-
      Distance, in metres, from the current global-plan endpoint within which
      in-place rotation counts as terminal alignment. Terminal alignment may
      turn at most max(0.5, initial yaw error + 0.5) rad. Should not exceed
      the goal checker's XY tolerance.
  guard_terminal_timeout_s:
    type: number
    default: 15.0
    minimum: 0
    x-group: Guard
    description: >-
      Maximum time, in seconds, from the first in-place rotation inside
      guard_terminal_xy_m until the goal is stopped. Pauses in rotation do
      not reset it; only a new goal does.
  guard_no_progress_s:
    type: number
    default: 3.0
    minimum: 0
    x-group: Guard
    description: >-
      Maximum time, in seconds, during terminal alignment without the yaw
      error to the plan endpoint dropping by more than 0.04 rad before the
      goal is stopped.
  guard_global_spin_timeout_s:
    type: number
    default: 25.0
    minimum: 0
    x-group: Guard
    description: >-
      Maximum time, in seconds, an in-place rotation anywhere on the route,
      including recovery loops, may continue without odometry yaw advancing
      by at least 0.04 rad. The timer restarts on each such advance.
  guard_global_spin_limit_rad:
    type: number
    default: 6.783185307
    minimum: 0
    x-group: Guard
    description: >-
      Maximum cumulative odometry yaw, in radians, during one continuous
      in-place rotation before the goal is stopped. The default is one full
      turn plus 0.5 rad.

  trajectory_log_dir:
    type: string
    x-group: Diagnostics
    description: >-
      Directory for per-goal JSONL trajectories and scan-anomaly records,
      written by the velocity guard. Defaults to rbnx-build/data/trajectories
      under the package root; a relative path is relative to the package
      root. In Docker mode the package directory is mounted at /nav2 and is
      the only writable host mount, so use a relative path there; any other
      path is lost when the container exits.

  params_profile:
    type: string
    enum: [default, slam, sim, ranger_mini_v3]
    x-group: Deprecated
    description: >-
      Deprecated: use params_file. Selects a parameter file packaged with this
      provider and logs a migration warning. An unknown value fails
      initialization even when params_file is set. ranger_mini_v3 also
      supplies its packaged BehaviorTree when bt_xml_file is absent.

required: [dynamic_speed]
