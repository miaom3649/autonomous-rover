"""
Depth bridge — runs on the Raspberry Pi.

Monitors /rover/cmd_vel for stop events, grabs one frame per stop,
POSTs it to the Windows depth server, and publishes the metric depth
map as /rover/camera/depth (32FC1, meters) with the source frame's
original timestamp so ORB-SLAM3 RGBD sync works correctly.

The AI depth model is only scale-approximate (monocular metric depth is an
ill-posed problem — see README for measured error). When a fresh forward-facing
lidar reading is available, it's used as a trusted anchor: we compare it against
the AI's estimate for the forward-facing center of the frame and rescale the
whole depth map by that ratio before publishing. The 2D lidar (mounted for SLAM)
does double duty here — its narrow per-degree beam pins down exactly which
point it's measuring, unlike the ultrasonic sensor's wide cone ("distance to
whatever's nearest somewhere in this broad area"), which made it impossible to
know for sure which pixel a given reading corresponded to.
"""
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32


class DepthBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_bridge_node")

        self.declare_parameter("depth_server_url", "http://192.168.1.100:8765/depth")
        self.declare_parameter("settle_delay", 0.5)
        self.declare_parameter("request_timeout", 0.8)
        self.declare_parameter("lidar_correction", False)
        self.declare_parameter("lidar_max_age", 1.0)
        self.declare_parameter("depth_region_frac", 0.2)
        # Angle (radians, in the lidar's own frame) that points straight ahead,
        # matching the camera's optical axis. 0.0 assumes the lidar's zero
        # heading is already aligned with "forward" — adjust after mounting
        # if the two aren't lined up.
        self.declare_parameter("lidar_forward_angle_rad", 0.0)
        # Half-width of the angular window (radians) averaged around the
        # forward angle, for a little robustness against single-point noise.
        self.declare_parameter("lidar_sample_halfwidth_rad", 0.03)

        self._server_url: str = self.get_parameter("depth_server_url").value
        self._settle_delay: float = float(self.get_parameter("settle_delay").value)
        self._timeout: float = float(self.get_parameter("request_timeout").value)
        self._lidar_correction: bool = bool(self.get_parameter("lidar_correction").value)
        self._lidar_max_age: float = float(self.get_parameter("lidar_max_age").value)
        self._region_frac: float = float(self.get_parameter("depth_region_frac").value)
        self._lidar_forward_angle: float = float(
            self.get_parameter("lidar_forward_angle_rad").value
        )
        self._lidar_halfwidth: float = float(
            self.get_parameter("lidar_sample_halfwidth_rad").value
        )

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame: Optional[Image] = None
        self._last_moving_time: float = time.monotonic()
        self._processing: bool = False

        self._lidar_range: Optional[float] = None
        self._lidar_stamp: float = 0.0

        self._sub_cmd = self.create_subscription(
            Twist, "/rover/cmd_vel", self._on_cmd_vel, 10
        )
        self._sub_img = self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self._sub_scan = self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self._pub_depth = self.create_publisher(
            Image, "/rover/camera/depth", qos_profile_sensor_data
        )
        self._pub_scale = self.create_publisher(
            Float32, "/rover/camera/depth_correction_scale", qos_profile_sensor_data
        )

        self.create_timer(0.1, self._check_stop)

        self.get_logger().info(f"depth_bridge_node ready  server={self._server_url}")

    def _on_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        lo = self._lidar_forward_angle - self._lidar_halfwidth
        hi = self._lidar_forward_angle + self._lidar_halfwidth
        i_lo = max(0, int((lo - msg.angle_min) / msg.angle_increment))
        i_hi = min(len(msg.ranges) - 1, int((hi - msg.angle_min) / msg.angle_increment))
        if i_hi < i_lo:
            return

        window = np.array(msg.ranges[i_lo:i_hi + 1], dtype=np.float32)
        valid = window[
            np.isfinite(window) & (window >= msg.range_min) & (window <= msg.range_max)
        ]
        if valid.size == 0:
            return

        with self._lock:
            self._lidar_range = float(valid.mean())
            self._lidar_stamp = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            with self._lock:
                self._last_moving_time = time.monotonic()

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._latest_frame = msg

    def _check_stop(self) -> None:
        with self._lock:
            elapsed = time.monotonic() - self._last_moving_time
            frame = self._latest_frame
            if elapsed < self._settle_delay or frame is None or self._processing:
                return
            self._processing = True
            self._latest_frame = None  # consume — prevents double-firing this stop cycle

        t = threading.Thread(target=self._fetch_and_publish, args=(frame,), daemon=True)
        t.start()

    def _apply_lidar_correction(
        self, depth: np.ndarray
    ) -> tuple[np.ndarray, Optional[float]]:
        """Rescale `depth` so its forward-center estimate matches a fresh lidar reading.

        Monocular depth is scale-ambiguous — the AI's numbers can be systematically
        off by a ratio that varies with the scene (see README). The lidar's
        forward-facing reading gives one trusted point measurement of what's
        directly ahead; we use it to estimate that ratio for this frame and apply
        it to the whole map. Falls back to the raw (uncorrected) depth whenever
        there's no usable lidar reading. The lidar reading is trusted
        unconditionally otherwise — no plausibility check against the AI's own
        estimate, so a mismatch will scale the whole map by whatever ratio
        results, correct or not.
        Returns (possibly-corrected depth, scale factor used or None if unchanged).
        """
        if not self._lidar_correction:
            return depth, None

        with self._lock:
            lidar_range = self._lidar_range
            lidar_age = time.monotonic() - self._lidar_stamp

        if lidar_range is None or lidar_age > self._lidar_max_age:
            return depth, None

        h, w = depth.shape
        half_h = max(1, int(h * self._region_frac / 2))
        half_w = max(1, int(w * self._region_frac / 2))
        cy, cx = h // 2, w // 2
        region = depth[cy - half_h:cy + half_h, cx - half_w:cx + half_w]
        valid = region[(region > 0) & np.isfinite(region)]
        if valid.size < region.size * 0.1:
            return depth, None

        ai_center_estimate = float(valid.mean())
        if ai_center_estimate <= 0:
            return depth, None

        scale = lidar_range / ai_center_estimate
        return (depth * scale).astype(np.float32), scale

    def _fetch_and_publish(self, frame_msg: Image) -> None:
        try:
            rgb = self._bridge.imgmsg_to_cv2(frame_msg, "rgb8")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            _, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])

            resp = requests.post(
                self._server_url,
                data=bytes(enc),
                headers={"Content-Type": "image/jpeg"},
                timeout=self._timeout,
            )
            resp.raise_for_status()

            h = int(resp.headers["X-Depth-Height"])
            w = int(resp.headers["X-Depth-Width"])
            depth = np.frombuffer(resp.content, dtype=np.float32).reshape(h, w)
            depth, scale = self._apply_lidar_correction(depth)
            if scale is not None:
                self.get_logger().debug(f"lidar correction: scale={scale:.3f}")
                self._pub_scale.publish(Float32(data=scale))

            depth_msg = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_msg.header = frame_msg.header
            self._pub_depth.publish(depth_msg)

            self.get_logger().debug(
                f"depth {w}x{h}  min={depth.min():.2f}m  max={depth.max():.2f}m"
            )
        except requests.Timeout:
            self.get_logger().warn("depth server timeout — skipping this stop cycle")
        except Exception as exc:
            self.get_logger().error(f"depth fetch failed: {exc}")
        finally:
            with self._lock:
                self._processing = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
