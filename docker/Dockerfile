# SPDX-License-Identifier: Apache-2.0
# nav2_wrapper_rbnx — Nav2 navigation service image.
#
# Why a container: nav2_wrapper drives a system-installed Nav2 stack, but
# many hosts (x86 dev boxes, the webots CI machine) have no host ROS2 at
# all. This image bundles ROS2 Humble + nav2_bringup so the wrapper runs
# anywhere docker does — the same portability story as mapping_rbnx. The
# native (host-ROS2) path stays available via scripts/start_native.sh for
# robots whose host already runs ROS2.
#
# --network host + --ipc=host at runtime so DDS/SHM line up with whatever
# sim/robot container publishes /map, /odom, /scan and consumes /cmd_vel.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ── TUNA mirrors for domestic (CN) networks ─────────────────────────────
RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
            -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
            /etc/apt/sources.list; \
    fi
ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

# Nav2 stack + tf + the pointcloud→laserscan adapter (for 3D-lidar-only
# robots whose costmap wants a LaserScan).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-yaml \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-tf2-ros \
        ros-humble-pointcloud-to-laserscan \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
        grpcio>=1.60.0 \
        mcp>=1.0 \
        fastmcp>=3 \
        protobuf>=4.25.0 \
        pyyaml>=6.0

WORKDIR /nav2
COPY entrypoint.sh /entrypoint.sh
COPY no_shm_profile.xml /etc/fastrtps_no_shm.xml
RUN chmod +x /entrypoint.sh

# Match the sim's RMW + force UDP-only (SHM across containers needs
# --ipc=host; UDP is the reliable cross-container path).
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastrtps_no_shm.xml \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ENTRYPOINT ["/entrypoint.sh"]
