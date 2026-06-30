#!/usr/bin/env python3
"""
Live camera viewer with ORB feature overlay.
Run on the Pi while the SLAM stack (or just camera_node) is running:

    python3 scripts/camera_viewer.py

Then open http://raspberrypi.local:8080 in your browser on the dev machine.
"""
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from http.server import BaseHTTPRequestHandler, HTTPServer

_latest_jpeg: bytes | None = None
_lock = threading.Lock()
PORT = 8080


class _MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        import time
        try:
            while True:
                with _lock:
                    data = _latest_jpeg
                if data:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except Exception:
            pass


class CameraViewerNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_viewer")
        self._bridge = CvBridge()
        self._orb = cv2.ORB_create(nfeatures=1000, fastThreshold=3)
        self.create_subscription(
            Image, "/rover/camera/image_raw", self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(f"Camera viewer ready — open http://raspberrypi.local:{PORT}")

    def _on_image(self, msg: Image) -> None:
        global _latest_jpeg
        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        kps = self._orb.detect(frame)
        cv2.drawKeypoints(frame, kps, frame, color=(0, 255, 0),
                          flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.putText(frame, f"features: {len(kps)}", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with _lock:
            _latest_jpeg = jpeg.tobytes()


def main() -> None:
    rclpy.init()
    node = CameraViewerNode()

    server = HTTPServer(("0.0.0.0", PORT), _MjpegHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()


if __name__ == "__main__":
    main()
