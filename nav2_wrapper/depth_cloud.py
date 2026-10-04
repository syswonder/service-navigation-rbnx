"""A depth camera as a PointCloud2 the costmaps and Collision Monitor can mark.

A 2D lidar sees one plane; a table top, a shelf or anything overhanging the
floor stays outside it until the robot's body meets it. The depth camera sees
them. This node back-projects the deployment's depth image with the camera's
own intrinsics (the Robonix `camera/intrinsics` contract, a latched
CameraInfo) into a sparse cloud in the camera frame; costmap layers place it
through TF and keep only the heights that matter to the robot.

depth_image_proc would need a CameraInfo stamped with every frame; the
intrinsics contract publishes one, once. Sampling every Nth pixel and capping
the rate also keeps the cloud small enough for a voxel layer to stay cheap.
"""

from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthCloud(Node):
    def __init__(self):
        super().__init__("robonix_depth_cloud")
        self.stride = max(1, int(os.getenv("ROBONIX_DEPTH_STRIDE", "4")))
        self.min_range = float(os.getenv("ROBONIX_DEPTH_MIN_RANGE_M", "0.2"))
        self.max_range = float(os.getenv("ROBONIX_DEPTH_MAX_RANGE_M", "4.0"))
        self.period = 1.0 / max(0.1, float(os.getenv("ROBONIX_DEPTH_RATE_HZ", "10")))
        self.info: CameraInfo | None = None
        self._last = 0.0
        self._grid = None
        self._pub = self.create_publisher(PointCloud2, os.environ["ROBONIX_DEPTH_CLOUD_OUT"], qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, os.environ["ROBONIX_DEPTH_INFO_TOPIC"], self._on_info, latched)
        self.create_subscription(Image, os.environ["ROBONIX_DEPTH_IMAGE_TOPIC"], self._on_depth, qos_profile_sensor_data)
        self.get_logger().info(
            f"depth cloud: stride={self.stride} range=[{self.min_range}, {self.max_range}] m "
            f"rate={1 / self.period:.0f} Hz"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg
        self._grid = None

    def _rays(self, h: int, w: int):
        """Per sampled pixel, the (x/z, y/z) the camera model gives it."""
        if self._grid is None or self._grid[0] != (h, w):
            fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
            # Intrinsics may describe a different resolution than the image.
            sx = w / self.info.width if self.info.width else 1.0
            sy = h / self.info.height if self.info.height else 1.0
            us = np.arange(0, w, self.stride, dtype=np.float32)
            vs = np.arange(0, h, self.stride, dtype=np.float32)
            u, v = np.meshgrid(us, vs)
            self._grid = ((h, w), (u - cx * sx) / (fx * sx), (v - cy * sy) / (fy * sy))
        return self._grid[1], self._grid[2]

    def _on_depth(self, msg: Image) -> None:
        now = time.monotonic()
        if self.info is None or now - self._last < self.period:
            return
        self._last = now
        if msg.encoding in ("32FC1", "32fc1"):
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, : msg.width]
        elif msg.encoding in ("16UC1", "mono16"):
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width] * np.float32(0.001)
        else:
            self.get_logger().warn(f"unsupported depth encoding {msg.encoding}", throttle_duration_sec=30.0)
            return
        z = depth[:: self.stride, :: self.stride].astype(np.float32)
        rx, ry = self._rays(msg.height, msg.width)
        keep = np.isfinite(z) & (z >= self.min_range) & (z <= self.max_range)
        xyz = np.stack((rx[keep] * z[keep], ry[keep] * z[keep], z[keep]), axis=-1).astype(np.float32)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(xyz.shape[0])
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * out.width
        out.is_dense = True
        out.data = xyz.tobytes()
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DepthCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
