"""Maintain and publish the persistent semantic-object database."""

import json
from pathlib import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from rover_navigation.semantic_store import SemanticStore


class SemanticMapperNode(Node):
    def __init__(self):
        super().__init__("semantic_mapper_node")
        self.declare_parameter("confirm_observations", 3)
        self.declare_parameter("association_radius_m", 0.35)
        self._store: SemanticStore | None = None
        self._session_dir: str | None = None
        self._accept_observations = False
        self._visible = False
        self._pub = self.create_publisher(String, "/rover/semantic_objects", 10)
        self.create_subscription(String, "/rover/localized_objects", self._on_objects, 10)
        self.create_subscription(String, "/rover/mapping_status", self._on_mapping_status, 10)
        self.create_timer(5.0, self._maintenance)

    def _on_mapping_status(self, msg):
        try:
            status = json.loads(msg.data)
            state = status.get("state")
            session_dir = status.get("session_dir")
            if session_dir and session_dir != self._session_dir:
                self._open_session_store(session_dir)
            # Geometric mapping must be finalized first. YOLO may continue to
            # draw camera boxes, but semantic positions are persisted only
            # after the operator explicitly enters work mode.
            self._accept_observations = state == "WORKING" and self._store is not None
            self._visible = self._store is not None and state not in (
                "INVALID", "RECOVERY_FAILED", "RETURN_TO_CHECKPOINT"
            )
            self._publish()
        except (TypeError, ValueError):
            pass

    def _open_session_store(self, session_dir: str) -> None:
        if self._store is not None:
            self._store.close()
        path = Path(session_dir).expanduser() / "semantic_objects.sqlite3"
        path.parent.mkdir(parents=True, exist_ok=True)
        self._store = SemanticStore(
            str(path), int(self.get_parameter("confirm_observations").value),
            float(self.get_parameter("association_radius_m").value),
        )
        self._session_dir = session_dir

    def _on_objects(self, msg):
        if not self._accept_observations or self._store is None:
            return
        try:
            for obj in json.loads(msg.data):
                self._store.observe(obj["class"], obj["x"], obj["y"], obj["confidence"],
                                    obj["lidar_confirmed"], obj.get("stamp"))
            self._publish()
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"Rejected localized objects: {exc}")

    def _maintenance(self):
        if self._store is None:
            self._publish()
            return
        self._store.expire_candidates()
        self._store.expire_classes(("person", "dog", "cat"), ttl_s=5.0)
        self._publish()

    def _publish(self):
        objects = self._store.all() if self._visible and self._store is not None else []
        self._pub.publish(String(data=json.dumps(objects)))

    def destroy_node(self):
        if self._store is not None:
            self._store.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = SemanticMapperNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.shutdown()
