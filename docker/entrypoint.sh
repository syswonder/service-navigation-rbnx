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
cd /nav2

# Generated atlas_pb2 etc. live under <pkg>/rbnx-build/codegen/proto_gen
# (rbnx codegen default). robonix-api is bind-mounted at /robonix-api.
export PYTHONPATH="/nav2:/nav2/rbnx-build/codegen/proto_gen:/nav2/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
if [ -d /robonix-api ]; then
    export PYTHONPATH="/robonix-api:${PYTHONPATH}"
fi

mkdir -p /nav2/rbnx-build/data

exec python3 -m nav2_wrapper.atlas_bridge
