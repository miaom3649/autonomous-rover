"""Translate a semantic class goal into a safe Nav2 NavigateToPose goal."""

import json
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class SemanticNavigationNode(Node):
    def __init__(self):
        super().__init__("semantic_navigation_node")
        self.declare_parameter("stand_off_distance_m", 0.5)
        self._objects = []
        self._map = None
        self._tf = Buffer(); self._listener = TransformListener(self._tf, self)
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status = self.create_publisher(String, "/rover/semantic_navigation_status", 10)
        self.create_subscription(String, "/rover/semantic_objects", self._on_objects, 10)
        self.create_subscription(String, "/rover/semantic_goal", self._on_goal, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)

    def _on_objects(self, msg):
        try: self._objects = json.loads(msg.data)
        except ValueError: pass

    def _on_map(self, msg): self._map = msg

    def _on_goal(self, msg):
        label = msg.data.strip().lower()
        try:
            transform = self._tf.lookup_transform("map", "base_link", rclpy.time.Time())
            rx, ry = transform.transform.translation.x, transform.transform.translation.y
            candidates = [obj for obj in self._objects
                          if obj["class"].lower() == label and obj["status"] == "confirmed"]
            target = min(candidates, key=lambda obj: math.hypot(obj["x"]-rx, obj["y"]-ry))
        except (ValueError, KeyError):
            self._status.publish(String(data=json.dumps({"status": "NO_TARGET", "class": label})))
            return
        distance = math.hypot(target["x"]-rx, target["y"]-ry)
        if distance < 1e-6: return
        stand_off = min(float(self.get_parameter("stand_off_distance_m").value), distance*0.8)
        preferred = math.atan2(ry-target["y"], rx-target["x"])
        candidates = [preferred + offset for offset in (0, .52, -.52, 1.05, -1.05, math.pi)]
        free = [(target["x"]+stand_off*math.cos(angle),
                 target["y"]+stand_off*math.sin(angle)) for angle in candidates]
        free = [point for point in free if self._is_free(*point)]
        if not free:
            self._status.publish(String(data=json.dumps({"status": "NO_SAFE_APPROACH",
                                                         "object_id": target["id"]})))
            return
        gx, gy = min(free, key=lambda point: math.hypot(point[0]-rx, point[1]-ry))
        yaw = math.atan2(target["y"]-gy, target["x"]-gx)
        goal = NavigateToPose.Goal(); goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self._client.send_goal_async(goal)
        self._status.publish(String(data=json.dumps({"status": "SENT", "class": label,
            "object_id": target["id"], "goal_x": gx, "goal_y": gy})))

    def _is_free(self, x, y):
        grid = self._map
        if grid is None: return False
        col = int((x-grid.info.origin.position.x)/grid.info.resolution)
        row = int((y-grid.info.origin.position.y)/grid.info.resolution)
        if not (0 <= col < grid.info.width and 0 <= row < grid.info.height): return False
        value = grid.data[row*grid.info.width+col]
        return 0 <= value < 50


def main(args=None):
    rclpy.init(args=args); node = SemanticNavigationNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.shutdown()
