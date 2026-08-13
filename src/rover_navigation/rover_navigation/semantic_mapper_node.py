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
        self.declare_parameter("database_path", "mapping_sessions/semantic_objects.sqlite3")
        self.declare_parameter("confirm_observations", 3)
        self.declare_parameter("association_radius_m", 0.35)
        path = Path(str(self.get_parameter("database_path").value)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._store = SemanticStore(str(path), int(self.get_parameter("confirm_observations").value),
                                    float(self.get_parameter("association_radius_m").value))
        self._enabled = True
        self._pub = self.create_publisher(String, "/rover/semantic_objects", 10)
        self.create_subscription(String, "/rover/localized_objects", self._on_objects, 10)
        self.create_subscription(String, "/rover/mapping_status", self._on_mapping_status, 10)
        self.create_timer(5.0, self._maintenance)

    def _on_mapping_status(self, msg):
        try:
            self._enabled = json.loads(msg.data).get("state") not in (
                "INVALID", "RECOVERY_FAILED", "RETURN_TO_CHECKPOINT")
        except ValueError: pass

    def _on_objects(self, msg):
        if not self._enabled: return
        try:
            for obj in json.loads(msg.data):
                self._store.observe(obj["class"], obj["x"], obj["y"], obj["confidence"],
                                    obj["lidar_confirmed"], obj.get("stamp"))
            self._publish()
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"Rejected localized objects: {exc}")

    def _maintenance(self):
        self._store.expire_candidates()
        self._store.expire_classes(("person", "dog", "cat"), ttl_s=5.0)
        self._publish()
    def _publish(self): self._pub.publish(String(data=json.dumps(self._store.all())))
    def destroy_node(self): self._store.close(); super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = SemanticMapperNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.shutdown()
