import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveNode(Node):
    """Subscribe to /rover/cmd_vel and drive PiCar-X motors."""

    def __init__(self) -> None:
        super().__init__("drive_node")

        self.declare_parameter("use_sim", False)
        self.declare_parameter("max_linear_vel", 0.5)     # m/s at full throttle
        self.declare_parameter("max_angular_vel", 2.0)    # rad/s at full lock
        self.declare_parameter("max_motor_speed", 50)     # SDK scale 0–100
        self.declare_parameter("max_steering_angle", 30.0)  # degrees
        self.declare_parameter("cmd_timeout", 0.5)        # stop if no cmd for this long

        self._use_sim = self.get_parameter("use_sim").as_bool()
        self._max_linear = self.get_parameter("max_linear_vel").as_double()
        self._max_angular = self.get_parameter("max_angular_vel").as_double()
        self._max_speed = self.get_parameter("max_motor_speed").as_integer()
        self._max_angle = self.get_parameter("max_steering_angle").as_double()
        self._cmd_timeout = self.get_parameter("cmd_timeout").as_double()

        if not self._use_sim:
            from picarx import Picarx  # type: ignore[import]
            self._px = Picarx()
        else:
            self._px = None

        self._sub = self.create_subscription(Twist, "/rover/cmd_vel", self._on_cmd_vel, 10)
        self._last_cmd = self.get_clock().now()
        # Check at 10 Hz whether the command stream has gone silent
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(f"Drive node ready (sim={self._use_sim})")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = self.get_clock().now()
        self._drive(msg.linear.x, msg.angular.z)

    def _drive(self, linear_x: float, angular_z: float) -> None:
        speed = int(
            max(-self._max_speed,
                min(self._max_speed,
                    linear_x / self._max_linear * self._max_speed))
        )
        angle = max(-self._max_angle,
                    min(self._max_angle,
                        angular_z / self._max_angular * self._max_angle))

        if self._use_sim:
            self.get_logger().debug(f"[sim] speed={speed} angle={angle:.1f}°")
            return

        self._px.set_dir_servo_angle(angle)
        if speed > 0:
            self._px.forward(speed)
        elif speed < 0:
            self._px.backward(-speed)
        else:
            self._px.stop()

    def _watchdog(self) -> None:
        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if elapsed > self._cmd_timeout:
            self._drive(0.0, 0.0)

    def destroy_node(self) -> None:
        if self._px is not None:
            self._px.stop()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
