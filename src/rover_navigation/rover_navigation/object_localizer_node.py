"""Project YOLO detections to the map and snap them to nearby LiDAR points."""

import json
import math
import time
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rover_navigation.ground_projection import pixel_to_ground


class ObjectLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("object_localizer_node")
        defaults = {
            "camera_fx": 0.0, "camera_fy": 0.0, "camera_cx": 0.0, "camera_cy": 0.0,
            "camera_k1": 0.0, "camera_k2": 0.0, "camera_p1": 0.0, "camera_p2": 0.0,
            "camera_height_m": 0.0, "camera_x_m": 0.0, "camera_y_m": 0.0,
            "camera_pitch_down_deg": 0.0, "camera_yaw_left_deg": 0.0,
            "image_width": 640.0, "image_height": 480.0, "lidar_match_radius_m": 0.35,
            "lidar_x_m": 0.0, "lidar_y_m": 0.0, "lidar_yaw_rad": 0.0,
        }
        for name, default in defaults.items(): self.declare_parameter(name, default)
        self._scans = deque(maxlen=30)
        self._tf = Buffer(); self._listener = TransformListener(self._tf, self)
        self._publisher = self.create_publisher(String, "/rover/localized_objects", 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/rover/object_detections", self._on_detections, 10)

    def _on_scan(self, scan):
        stamp_ns = int(scan.header.stamp.sec)*1_000_000_000 + int(scan.header.stamp.nanosec)
        self._scans.append((stamp_ns, scan))

    def _on_detections(self, message):
        try:
            detections = json.loads(message.data)
            stamp_ns = int(detections[0].get("image_stamp_ns", 0)) if detections else 0
            lookup_time = rclpy.time.Time(nanoseconds=stamp_ns) if stamp_ns else rclpy.time.Time()
            transform = self._tf.lookup_transform("map", "base_link", lookup_time)
            q = transform.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            output = []
            for detection in detections:
                if not detection.get("project_to_ground"): continue
                camera_point = self._camera_point(detection)
                if camera_point is None: continue
                point, lidar = self._match_lidar(camera_point, stamp_ns)
                x = transform.transform.translation.x + math.cos(yaw)*point[0]-math.sin(yaw)*point[1]
                y = transform.transform.translation.y + math.sin(yaw)*point[0]+math.cos(yaw)*point[1]
                output.append({"class": detection["label"], "confidence": detection["confidence"],
                    "x": x, "y": y, "camera_map_x": transform.transform.translation.x +
                    math.cos(yaw)*camera_point[0]-math.sin(yaw)*camera_point[1],
                    "camera_map_y": transform.transform.translation.y +
                    math.sin(yaw)*camera_point[0]+math.cos(yaw)*camera_point[1],
                    "lidar_confirmed": lidar, "stamp": time.time(),
                    "image_stamp_ns": detection.get("image_stamp_ns")})
            self._publisher.publish(String(data=json.dumps(output)))
        except Exception as exc:
            self.get_logger().warn(f"Could not localize detections: {exc}")

    def _camera_point(self, detection):
        p = {name: float(self.get_parameter(name).value) for name in (
            "camera_fx", "camera_fy", "camera_cx", "camera_cy", "camera_k1", "camera_k2",
            "camera_p1", "camera_p2", "camera_height_m", "camera_x_m", "camera_y_m",
            "camera_pitch_down_deg", "camera_yaw_left_deg", "image_width", "image_height")}
        raw = np.array([[[(detection["x1"]+detection["x2"])*0.5*p["image_width"],
                          detection["y2"]*p["image_height"]]]], dtype=np.float64)
        matrix = np.array([[p["camera_fx"],0,p["camera_cx"]],
                           [0,p["camera_fy"],p["camera_cy"]],[0,0,1]])
        distortion = np.array([p["camera_k1"],p["camera_k2"],p["camera_p1"],p["camera_p2"]])
        u, v = cv2.undistortPoints(raw, matrix, distortion, P=matrix)[0, 0]
        result = pixel_to_ground(u, v, fx=p["camera_fx"], fy=p["camera_fy"],
            cx=p["camera_cx"], cy=p["camera_cy"], camera_height_m=p["camera_height_m"],
            camera_x_m=p["camera_x_m"], camera_y_m=p["camera_y_m"],
            camera_pitch_down_deg=p["camera_pitch_down_deg"],
            camera_yaw_left_deg=p["camera_yaw_left_deg"])
        return result[:2] if result else None

    def _match_lidar(self, point, stamp_ns):
        if not self._scans: return point, False
        _, scan = min(self._scans, key=lambda item: abs(item[0]-stamp_ns)) if stamp_ns else self._scans[-1]
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        laser_x, laser_y = ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])
        lidar_yaw = float(self.get_parameter("lidar_yaw_rad").value)
        offset_x = float(self.get_parameter("lidar_x_m").value)
        offset_y = float(self.get_parameter("lidar_y_m").value)
        xs = offset_x + np.cos(lidar_yaw)*laser_x - np.sin(lidar_yaw)*laser_y
        ys = offset_y + np.sin(lidar_yaw)*laser_x + np.cos(lidar_yaw)*laser_y
        nearby = np.hypot(xs-point[0], ys-point[1]) <= float(
            self.get_parameter("lidar_match_radius_m").value)
        if not np.any(nearby): return point, False
        return (float(np.median(xs[nearby])), float(np.median(ys[nearby]))), True


def main(args=None):
    rclpy.init(args=args); node = ObjectLocalizerNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.shutdown()
