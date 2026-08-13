"""Monitor SLAM health, persist sessions, and control the mapping safety lock."""

import json
import math
from pathlib import Path
import subprocess
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from rover_navigation.mapping_health import HealthSample, HealthThresholds, evaluate_sample

_LOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MappingMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_monitor_node")
        for name, default in (
            ("sample_period_s", 0.5), ("pose_jump_m", 0.5), ("yaw_jump_deg", 35.0),
            ("scan_timeout_s", 1.0), ("map_timeout_s", 3.0),
            ("min_valid_scan_points", 30), ("failures_before_recovery", 3),
            ("healthy_samples_to_recover", 4), ("recovery_timeout_s", 5.0),
            ("checkpoint_period_s", 15.0),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("session_root", "mapping_sessions")

        self._thresholds = HealthThresholds(
            pose_jump_m=float(self.get_parameter("pose_jump_m").value),
            yaw_jump_deg=float(self.get_parameter("yaw_jump_deg").value),
            scan_timeout_s=float(self.get_parameter("scan_timeout_s").value),
            map_timeout_s=float(self.get_parameter("map_timeout_s").value),
            min_valid_scan_points=int(self.get_parameter("min_valid_scan_points").value),
        )
        self._state = "IDLE"
        self._scan: LaserScan | None = None
        self._scan_time = 0.0
        self._map_time = 0.0
        self._previous: HealthSample | None = None
        self._failure_count = 0
        self._healthy_count = 0
        self._recovery_started = 0.0
        self._last_checkpoint = 0.0
        self._session_dir: Path | None = None
        self._summary: dict = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._status_pub = self.create_publisher(String, "/rover/mapping_status", 10)
        self._lock_pub = self.create_publisher(Bool, "/rover/motion_lock", _LOCK_QOS)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos_profile_sensor_data)
        self.create_subscription(String, "/rover/mapping_control", self._on_control, 10)
        self.create_timer(float(self.get_parameter("sample_period_s").value), self._sample)
        self._publish_status("monitor ready")

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan, self._scan_time = msg, time.monotonic()

    def _on_map(self, _msg: OccupancyGrid) -> None:
        self._map_time = time.monotonic()

    def _on_control(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "START":
            self._start_session()
        elif command == "FINISH" and self._state != "IDLE":
            self._record_event("MAPPING_FINISHED", [], "session saved")
            if self._session_dir:
                subprocess.Popen(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f",
                     str(self._session_dir / "final_map")],
                    stdout=(self._session_dir / "final_save.log").open("a"),
                    stderr=subprocess.STDOUT,
                )
            self._state = "IDLE"
            self._set_lock(False)
            self._flush_summary()
            self._publish_status("mapping finished")
        elif command == "ABORT" and self._state != "IDLE":
            self._record_event("MAPPING_ABORTED_BY_USER", [], "session aborted")
            self._state = "INVALID"
            self._set_lock(True)
            self._flush_summary()
            self._publish_status("mapping aborted")
        elif command == "RETURN" and self._state in ("INVALID", "RECOVERY_FAILED"):
            self._state = "RETURN_TO_CHECKPOINT"
            self._set_lock(False)
            self._record_event("RETURN_TO_CHECKPOINT", [], "manual driving enabled")
            self._publish_status("return manually; current map is invalid")
        elif command == "RESUME" and self._state == "RETURN_TO_CHECKPOINT":
            self._resume_checkpoint()

    def _start_session(self) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        root = Path(str(self.get_parameter("session_root").value)).expanduser()
        self._session_dir = root / f"session_{stamp}"
        self._session_dir.mkdir(parents=True, exist_ok=False)
        self._summary = {"session_id": self._session_dir.name, "map_id": self._session_dir.name,
                         "started_at": datetime.now(timezone.utc).isoformat(), "points": []}
        self._state, self._previous = "MAPPING", None
        self._failure_count = self._healthy_count = 0
        self._last_checkpoint = time.monotonic() - float(
            self.get_parameter("checkpoint_period_s").value
        )
        self._set_lock(False)
        self._record_event("MAPPING_STARTED", [], "monitoring enabled")
        self._publish_status("mapping started")

    def _resume_checkpoint(self) -> None:
        if not self._session_dir or not (self._session_dir / "checkpoint.json").exists():
            raise ValueError("no trusted checkpoint is available")
        checkpoint = json.loads((self._session_dir / "checkpoint.json").read_text(encoding="utf-8"))
        base = str(self._session_dir / checkpoint["posegraph_base"])
        pose = checkpoint
        # Humble DeserializePoseGraph: 2 = START_AT_GIVEN_POSE. The operator
        # has physically returned the rover near the recorded checkpoint pose.
        request = ("{filename: '" + base + "', match_type: 2, initial_pose: {x: " +
                   str(pose["x"]) + ", y: " + str(pose["y"]) + ", theta: " +
                   str(math.radians(pose["yaw_deg"])) + "}}")
        subprocess.Popen(
            ["ros2", "service", "call", "/slam_toolbox/deserialize_map",
             "slam_toolbox/srv/DeserializePoseGraph", request],
            stdout=(self._session_dir / "checkpoint_service.log").open("a"),
            stderr=subprocess.STDOUT,
        )
        self._state = "RECOVERING"
        self._previous = None
        self._failure_count = self._healthy_count = 0
        self._recovery_started = time.monotonic()
        self._set_lock(True)
        self._record_event("CHECKPOINT_RELOAD_REQUESTED", [], "relocalizing at trusted pose")
        self._publish_status("checkpoint reload requested")

    def _pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        q = transform.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y * q.y + q.z * q.z)))
        return transform.transform.translation.x, transform.transform.translation.y, yaw

    def _sample(self) -> None:
        if self._state not in ("MAPPING", "RECOVERING"):
            return
        now = time.monotonic()
        pose = self._pose()
        scan = self._scan
        valid = 0
        total = len(scan.ranges) if scan else 0
        if scan:
            ranges = np.asarray(scan.ranges)
            valid = int(np.count_nonzero(np.isfinite(ranges) &
                        (ranges >= scan.range_min) & (ranges <= scan.range_max)))
        sample = HealthSample(
            stamp=time.time(), x=pose[0] if pose else None, y=pose[1] if pose else None,
            yaw_deg=pose[2] if pose else None,
            scan_age_s=now - self._scan_time if self._scan_time else float("inf"),
            map_age_s=now - self._map_time if self._map_time else float("inf"),
            valid_scan_points=valid, total_scan_points=total, tf_ok=pose is not None,
        )
        reasons = evaluate_sample(sample, self._previous, self._thresholds)
        # Do not compare a post-fault pose against the known-bad jump forever.
        self._previous = sample
        self._record_sample(sample, reasons)

        if reasons:
            self._failure_count += 1
            self._healthy_count = 0
        else:
            self._failure_count = 0
            self._healthy_count += 1

        required = int(self.get_parameter("failures_before_recovery").value)
        if self._state == "MAPPING" and self._failure_count >= required:
            self._state = "RECOVERING"
            self._recovery_started = now
            self._set_lock(True)
            self._record_event("RECOVERY_STARTED", reasons, "motor locked; buffering lidar")
        elif self._state == "RECOVERING":
            if self._healthy_count >= int(self.get_parameter("healthy_samples_to_recover").value):
                self._state = "MAPPING"
                self._set_lock(False)
                self._record_event("RECOVERED", [], "motor unlocked; mapping resumed")
            elif now - self._recovery_started >= float(
                self.get_parameter("recovery_timeout_s").value
            ):
                self._state = "RECOVERY_FAILED"
                self._set_lock(True)
                self._record_event("RECOVERY_TIMEOUT", reasons,
                                   "mapping invalid; request manual return")

        if self._state == "MAPPING" and not reasons and now - self._last_checkpoint >= float(
            self.get_parameter("checkpoint_period_s").value
        ):
            self._save_checkpoint(sample)
            self._last_checkpoint = now
        self._publish_status(reasons[0]["code"] if reasons else "healthy")

    def _save_checkpoint(self, sample: HealthSample) -> None:
        if not self._session_dir:
            return
        payload = {"time": sample.stamp, "x": sample.x, "y": sample.y,
                   "yaw_deg": sample.yaw_deg, "posegraph_base": "checkpoint"}
        temporary = self._session_dir / "checkpoint.json.tmp"
        temporary.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        temporary.replace(self._session_dir / "checkpoint.json")
        base = str(self._session_dir / "checkpoint")
        # Use the installed ROS CLI so this node remains importable in the dev VM,
        # where slam_toolbox_msgs Python bindings are intentionally unavailable.
        subprocess.Popen(
            ["ros2", "service", "call", "/slam_toolbox/serialize_map",
             "slam_toolbox/srv/SerializePoseGraph", f"{{filename: '{base}'}}"],
            stdout=(self._session_dir / "checkpoint_service.log").open("a"),
            stderr=subprocess.STDOUT,
        )
        self._record_event("CHECKPOINT", [], "latest trusted checkpoint updated")

    def _record_sample(self, sample: HealthSample, reasons: list[dict]) -> None:
        status = "abnormal" if reasons else "healthy"
        record = {"event": "HEALTH_SAMPLE", "time": sample.stamp, "x": sample.x,
                  "y": sample.y, "yaw_deg": sample.yaw_deg, "status": status,
                  "reasons": reasons, "scan_age_s": sample.scan_age_s,
                  "map_age_s": sample.map_age_s, "valid_scan_points": sample.valid_scan_points,
                  "total_scan_points": sample.total_scan_points, "state": self._state}
        self._write_full(record)
        if self._summary:
            self._summary["points"].append(record)
            self._flush_summary()

    def _record_event(self, event: str, reasons: list[dict], action: str) -> None:
        pose = self._pose()
        record = {"event": event, "time": time.time(), "status": "event", "state": self._state,
                  "x": pose[0] if pose else None, "y": pose[1] if pose else None,
                  "yaw_deg": pose[2] if pose else None, "reasons": reasons, "action": action}
        self._write_full(record)
        if self._summary:
            self._summary["points"].append(record)
            self._flush_summary()

    def _write_full(self, record: dict) -> None:
        if self._session_dir:
            record = self._json_safe(record)
            with (self._session_dir / "full_log.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, allow_nan=False) + "\n")

    def _flush_summary(self) -> None:
        if not self._session_dir:
            return
        temporary = self._session_dir / "summary.json.tmp"
        temporary.write_text(json.dumps(self._json_safe(self._summary), indent=2,
                                        allow_nan=False), encoding="utf-8")
        temporary.replace(self._session_dir / "summary.json")

    @classmethod
    def _json_safe(cls, value):
        if isinstance(value, float) and not math.isfinite(value):
            return None
        if isinstance(value, dict):
            return {key: cls._json_safe(item) for key, item in value.items()}
        if isinstance(value, list):
            return [cls._json_safe(item) for item in value]
        return value

    def _set_lock(self, locked: bool) -> None:
        self._lock_pub.publish(Bool(data=locked))

    def _publish_status(self, detail: str) -> None:
        payload = {"state": self._state, "detail": detail,
                   "session_id": self._summary.get("session_id"),
                   "session_dir": str(self._session_dir) if self._session_dir else None,
                   "failure_count": self._failure_count}
        self._status_pub.publish(String(data=json.dumps(payload)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MappingMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
