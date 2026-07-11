"""
Depth bridge — runs on the Raspberry Pi.

Monitors /rover/cmd_vel for stop events, grabs one frame per stop,
POSTs it to the Windows depth server, and publishes the metric depth
map as /rover/camera/depth (32FC1, meters) with the source frame's
original timestamp so ORB-SLAM3 RGBD sync works correctly.

The AI depth model is only scale-approximate (monocular metric depth is an
ill-posed problem — see README for measured error). When a fresh ultrasonic
reading is available, it's used as a trusted anchor: we compare it against
the AI's estimate for the forward-facing center of the frame and rescale
the whole depth map by that ratio before publishing. The ultrasonic sensor
is fixed on the chassis and the camera is held at a fixed pan/tilt during
normal operation, so "center of frame" and "straight ahead" stay aligned.
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
from sensor_msgs.msg import Image, Range


class DepthBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_bridge_node")

        self.declare_parameter("depth_server_url", "http://192.168.1.100:8765/depth")
        self.declare_parameter("settle_delay", 0.5)
        self.declare_parameter("request_timeout", 0.8)
        self.declare_parameter("ultrasonic_correction", True)
        self.declare_parameter("ultrasonic_max_age", 1.0)
        self.declare_parameter("ultrasonic_region_frac", 0.2)
        self.declare_parameter("correction_scale_min", 0.5)
        self.declare_parameter("correction_scale_max", 2.0)

        self._server_url: str = self.get_parameter("depth_server_url").value
        self._settle_delay: float = float(self.get_parameter("settle_delay").value)
        self._timeout: float = float(self.get_parameter("request_timeout").value)
        self._ultrasonic_correction: bool = bool(
            self.get_parameter("ultrasonic_correction").value
        )
        self._ultrasonic_max_age: float = float(
            self.get_parameter("ultrasonic_max_age").value
        )
        self._region_frac: float = float(self.get_parameter("ultrasonic_region_frac").value)
        self._scale_min: float = float(self.get_parameter("correction_scale_min").value)
        self._scale_max: float = float(self.get_parameter("correction_scale_max").value)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame: Optional[Image] = None
        self._last_moving_time: float = time.monotonic()
        self._processing: bool = False

        self._ultrasonic_range: Optional[float] = None
        self._ultrasonic_stamp: float = 0.0
        self._ultrasonic_min_range: float = 0.02
        self._ultrasonic_max_range: float = 4.0

        self._sub_cmd = self.create_subscription(
            Twist, "/rover/cmd_vel", self._on_cmd_vel, 10
        )
        self._sub_img = self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self._sub_range = self.create_subscription(
            Range, "/rover/ultrasonic/range", self._on_range, qos_profile_sensor_data
        )
        self._pub_depth = self.create_publisher(
            Image, "/rover/camera/depth", qos_profile_sensor_data
        )

        self.create_timer(0.1, self._check_stop)

        self.get_logger().info(f"depth_bridge_node ready  server={self._server_url}")

    def _on_range(self, msg: Range) -> None:
        with self._lock:
            self._ultrasonic_range = float(msg.range)
            self._ultrasonic_stamp = time.monotonic()
            self._ultrasonic_min_range = float(msg.min_range)
            self._ultrasonic_max_range = float(msg.max_range)

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

    def _apply_ultrasonic_correction(
        self, depth: np.ndarray
    ) -> tuple[np.ndarray, Optional[float]]:
        """Rescale `depth` so its forward-center estimate matches a fresh ultrasonic reading.

        Monocular depth is scale-ambiguous — the AI's numbers can be systematically
        off by a ratio that varies with the scene (see README). The ultrasonic sensor
        gives one trusted point measurement of what's directly ahead; we use it to
        estimate that ratio for this frame and apply it to the whole map. Falls back
        to the raw (uncorrected) depth whenever there's no usable ultrasonic reading,
        or when the ultrasonic and AI estimates disagree by more than
        correction_scale_min/max — a mismatch that large usually means the ultrasonic
        cone caught something (e.g. a small low obstacle) the camera never saw, so the
        two sensors are measuring different things and blending them would only make
        the whole map worse, not better.
        Returns (possibly-corrected depth, scale factor used or None if unchanged).
        """
        if not self._ultrasonic_correction:
            return depth, None

        with self._lock:
            us_range = self._ultrasonic_range
            us_age = time.monotonic() - self._ultrasonic_stamp
            us_min = self._ultrasonic_min_range
            us_max = self._ultrasonic_max_range

        if us_range is None or us_age > self._ultrasonic_max_age:
            return depth, None
        # readings pinned at the sensor's own limits usually mean "no valid echo"
        if us_range <= us_min or us_range >= us_max:
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

        scale = us_range / ai_center_estimate
        if scale < self._scale_min or scale > self._scale_max:
            # Ultrasonic and AI disagree too much to be looking at the same thing —
            # applying this scale would corrupt the whole map, so skip correction.
            return depth, None
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
            depth, scale = self._apply_ultrasonic_correction(depth)
            if scale is not None:
                self.get_logger().debug(f"ultrasonic correction: scale={scale:.3f}")

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
