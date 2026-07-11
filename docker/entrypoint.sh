#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper_rbnx container entrypoint.
#
# One process: the atlas bridge. It registers the navigation capability,
# discovers map/odom/scan/cmd_vel via atlas, rewrites the Nav2 params with
# the resolved topics, and spawns `ros2 launch nav2_bringup
# navigation_launch.py` IN this container (which has ROS2 + Nav2). SIGTERM
# tears the bridge (and its nav2 child) down.
set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /nav2/rbnx-build/codegen/ros2_idl/install/setup.bash ]]; then
    source /nav2/rbnx-build/codegen/ros2_idl/install/setup.bash
fi

configure_zenoh_session() {
    if [ "${RMW_IMPLEMENTATION:-}" != "rmw_zenoh_cpp" ] || [ -z "${ROBONIX_ZENOH_ROUTER:-}" ]; then
        return 0
    fi
    local src="/opt/ros/${ROS_DISTRO:-humble}/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
    local dst="/tmp/robonix_zenoh_session.json5"
    if [ ! -f "$src" ]; then
        echo "[entrypoint] missing Zenoh session config: $src" >&2
        return 1
    fi
    local mode="${ROBONIX_ZENOH_MODE:-client}"
    sed \
        -e "s#mode: \"peer\"#mode: \"${mode}\"#" \
        -e "s#\"tcp/localhost:7447\"#\"${ROBONIX_ZENOH_ROUTER}\"#g" \
        "$src" > "$dst"
    if [ -n "${ROBONIX_ZENOH_LISTEN:-}" ]; then
        sed -i "s#\"tcp/localhost:0\"#\"${ROBONIX_ZENOH_LISTEN}\"#g" "$dst"
    fi
    export ZENOH_SESSION_CONFIG_URI="$dst"
    export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
    echo "[entrypoint] rmw_zenoh_cpp mode=${mode} router=${ROBONIX_ZENOH_ROUTER} listen=${ROBONIX_ZENOH_LISTEN:-<default>}"
}

configure_zenoh_session
cd /nav2

# Generated atlas_pb2 etc. live under <pkg>/rbnx-build/codegen/proto_gen
# (rbnx codegen default). robonix-api is bind-mounted at /robonix-api.
export PYTHONPATH="/nav2:/nav2/rbnx-build/codegen/proto_gen:/nav2/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if [ -d /robonix-api ]; then
    export PYTHONPATH="/robonix-api:${PYTHONPATH}"
fi

mkdir -p /nav2/rbnx-build/data

exec python3 -m nav2_wrapper.atlas_bridge
