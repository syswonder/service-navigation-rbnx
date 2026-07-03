#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper start phase. Two execution shapes (same pattern as
# mapping_rbnx):
#
#   1. docker  (default) — `docker run` against `robonix-nav2`, which
#       bundles ROS2 Humble + nav2_bringup. Works on any host with docker
#       even when the host has NO ROS2 (x86 dev boxes, the webots CI box).
#   2. native  — scripts/start_native.sh: the atlas bridge as a host
#       process against a host-installed ROS2 + Nav2. Preferred on a robot
#       whose host already runs ROS2 (avoids the container hop).
#
# Selection (operator-set env in the shell running rbnx boot/start; the
# cap config arrives via Driver(CMD_INIT) so it can't be read here):
#   ROBONIX_NAV2_FORCE=native|docker     # explicit hard pin
#   ROBONIX_NAV2_PLATFORM=<platform>     # match NATIVE_PLATFORMS
#   default → docker
set -eo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

NATIVE_PLATFORMS=("jetson_orin")
is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE=""
case "${ROBONIX_NAV2_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") ;;
    *) echo "[nav2/start] ROBONIX_NAV2_FORCE=${ROBONIX_NAV2_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_NAV2_PLATFORM:-}"; then MODE=native; else MODE=docker; fi
fi
echo "[nav2/start] mode=${MODE} (FORCE=${ROBONIX_NAV2_FORCE:-} PLATFORM=${ROBONIX_NAV2_PLATFORM:-})"

if [[ "$MODE" == "native" ]]; then
    exec bash "${PKG}/scripts/start_native.sh"
fi

# ── Docker path ─────────────────────────────────────────────────────────
set -u
CT="${ROBONIX_NAV2_CONTAINER:-robonix_nav2}"
IMG="${ROBONIX_NAV2_IMAGE:-robonix-nav2}"

cleanup() {
    docker stop "$CT" >/dev/null 2>&1 || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

docker rm -f "$CT" >/dev/null 2>&1 || true
mkdir -p rbnx-build/data

declare -a ZENOH_ARGS=()
if [[ -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    ZENOH_ARGS=(-e "ROBONIX_ZENOH_ROUTER=${ROBONIX_ZENOH_ROUTER}")
fi
if [[ -n "${ROBONIX_ZENOH_MODE:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_MODE=${ROBONIX_ZENOH_MODE}")
fi
if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_LISTEN=${ROBONIX_ZENOH_LISTEN}")
fi

# config arrives via Driver(CMD_INIT) over gRPC; the container's bridge
# binds NAV2_DRIVER_PORT and registers with atlas at ROBONIX_ATLAS.
exec docker run --rm \
    --name "$CT" \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-nav2}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    "${ZENOH_ARGS[@]}" \
    -e NAV2_DRIVER_PORT="${NAV2_DRIVER_PORT:-50235}" \
    -e NAV2_LOG_LEVEL="${NAV2_LOG_LEVEL:-info}" \
    -v "$(pwd)":/nav2 \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    -v "$(pwd)/docker/no_shm_profile.xml":/etc/fastrtps_no_shm.xml:ro \
    "$IMG"
