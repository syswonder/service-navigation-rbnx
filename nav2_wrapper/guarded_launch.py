"""Fail-closed patching for the distro Nav2 navigation launch file."""

from __future__ import annotations

import ast
import re


EXTERNAL_BEHAVIOR_SINK = (
    "/robonix/staged_nav2/behavior_cmd_vel_forbidden"
)


def _package_blocks(source: str, package: str) -> list[str]:
    marker = f"package='{package}'"
    starts: list[int] = []
    offset = 0
    while True:
        start = source.find(marker, offset)
        if start < 0:
            break
        starts.append(start)
        offset = start + len(marker)
    blocks = []
    for start in starts:
        end = source.find("package='", start + len(marker))
        blocks.append(source[start : len(source) if end < 0 else end])
    return blocks


def _require_exact_producer_layout(
    source: str,
    *,
    patched: bool,
    external_velocity_guard: bool = False,
) -> None:
    controller_remap = "remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])"
    behavior_remap = (
        (
            "remappings=remappings + "
            f"[('cmd_vel', '{EXTERNAL_BEHAVIOR_SINK}')])"
            if external_velocity_guard
            else "remappings=remappings + "
            "[('cmd_vel', 'cmd_vel_guard_input')])"
        )
        if patched
        else "remappings=remappings)"
    )
    smoother_remap = (
        "[('cmd_vel', 'cmd_vel_nav'), "
        f"('cmd_vel_smoothed', '{'cmd_vel_guard_input' if patched else 'cmd_vel'}')])"
    )
    producers = {
        "nav2_controller": (controller_remap, False),
        "nav2_behaviors": (
            behavior_remap,
            patched and not external_velocity_guard,
        ),
        "nav2_velocity_smoother": (smoother_remap, patched),
    }
    for package, (required_remap, owns_guard_input) in producers.items():
        blocks = _package_blocks(source, package)
        if len(blocks) != 2:
            raise RuntimeError(
                f"unsupported nav2 producer layout: expected exactly 2 {package} blocks"
            )
        for block in blocks:
            if block.count(required_remap) != 1:
                raise RuntimeError(
                    f"unsupported nav2 {package} velocity remap layout"
                )
            if not owns_guard_input and "cmd_vel_guard_input" in block:
                raise RuntimeError(
                    f"unsupported nav2 {package} guard-input ownership"
                )

    if patched:
        canonical_target = re.compile(
            r"\(\s*['\"][^'\"]+['\"]\s*,\s*"
            r"['\"]/?cmd_vel['\"]\s*\)"
        )
        if canonical_target.search(source):
            raise RuntimeError("generated nav2 launch retains canonical cmd_vel output")
        expected_guard_inputs = 2 if external_velocity_guard else 4
        if source.count("cmd_vel_guard_input") != expected_guard_inputs:
            raise RuntimeError("generated nav2 launch has unexpected guard input count")
        if source.count("('cmd_vel', 'cmd_vel_nav')") != 4:
            raise RuntimeError("generated nav2 launch has unexpected cmd_vel_nav count")
        if external_velocity_guard:
            if source.count(EXTERNAL_BEHAVIOR_SINK) != 2:
                raise RuntimeError(
                    "generated nav2 launch has unexpected behavior sink count"
                )


def patch_navigation_launch(
    source: str,
    *,
    external_velocity_guard: bool = False,
) -> str:
    """Route every Nav2 velocity producer through the guard and own a container.

    ``navigation_launch.py`` normally expects ``bringup_launch.py`` to create
    the component container.  This provider launches the navigation file
    directly, so its opt-in composition path must create that container in the
    same launch process.  Exact layout checks deliberately fail closed when a
    distro update changes the launch structure.
    """
    required_imports = (
        "from launch.conditions import IfCondition",
        "from launch_ros.actions import Node",
    )
    missing = [item for item in required_imports if item not in source]
    if missing:
        raise RuntimeError(
            "unsupported nav2 navigation launch imports: " + ", ".join(missing)
        )

    if not isinstance(external_velocity_guard, bool):
        raise RuntimeError("external_velocity_guard must be a boolean")
    _require_exact_producer_layout(source, patched=False)

    text = source
    old_behavior = "remappings=remappings)"
    behavior_marker = "package='nav2_behaviors'"
    next_marker = "package='nav2_bt_navigator'"
    search_from = 0
    behavior_destination = (
        EXTERNAL_BEHAVIOR_SINK
        if external_velocity_guard
        else "cmd_vel_guard_input"
    )
    for _ in range(2):
        behavior_start = text.find(behavior_marker, search_from)
        if behavior_start < 0:
            raise RuntimeError("unsupported nav2 behavior_server launch layout")
        behavior_end = text.find(next_marker, behavior_start)
        if behavior_end < 0:
            raise RuntimeError("unsupported nav2 behavior_server launch layout")
        behavior = text[behavior_start:behavior_end]
        if behavior.count(old_behavior) != 1:
            raise RuntimeError("unsupported nav2 behavior_server launch layout")
        behavior = behavior.replace(
            old_behavior,
            "remappings=remappings + "
            f"[('cmd_vel', '{behavior_destination}')])",
        )
        text = text[:behavior_start] + behavior + text[behavior_end:]
        search_from = behavior_start + len(behavior)

    old_smoother = "[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])"
    new_smoother = (
        "[('cmd_vel', 'cmd_vel_nav'), "
        "('cmd_vel_smoothed', 'cmd_vel_guard_input')])"
    )
    if text.count(old_smoother) != 2:
        raise RuntimeError("unsupported nav2 velocity_smoother launch layout")
    text = text.replace(old_smoother, new_smoother)
    _require_exact_producer_layout(
        text,
        patched=True,
        external_velocity_guard=external_velocity_guard,
    )

    description_marker = "    # Create the launch description and populate\n"
    if text.count(description_marker) != 1:
        raise RuntimeError("unsupported nav2 launch description layout")
    container_action = """    # The provider invokes navigation_launch.py directly.  Keep the
    # optional component container inside this launch process so SIGTERM and
    # process-group cleanup cover the container and every loaded component.
    robonix_nav2_container = Node(
        condition=IfCondition(use_composition),
        name=container_name,
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen')

"""
    text = text.replace(
        description_marker,
        container_action + description_marker,
    )

    add_actions_marker = (
        "    # Add the actions to launch all of the navigation nodes\n"
        "    ld.add_action(load_nodes)\n"
        "    ld.add_action(load_composable_nodes)"
    )
    if text.count(add_actions_marker) != 1:
        raise RuntimeError("unsupported nav2 launch action layout")
    text = text.replace(
        add_actions_marker,
        "    # Add the owned container before loading composed nodes.\n"
        "    ld.add_action(robonix_nav2_container)\n"
        "    # Add the actions to launch all of the navigation nodes\n"
        "    ld.add_action(load_nodes)\n"
        "    ld.add_action(load_composable_nodes)",
    )

    try:
        ast.parse(text, filename="guarded_navigation_launch.py")
    except SyntaxError as error:
        raise RuntimeError("generated guarded nav2 launch is invalid") from error
    return text
