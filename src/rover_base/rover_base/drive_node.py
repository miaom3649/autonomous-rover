import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float32


class DriveNode(Node):
    """Subscribe to /rover/cmd_vel and drive PiCar-X motors.

    Also publishes ultrasonic range on /rover/ultrasonic/range using the
    Picarx instance (avoids GPIO conflict with a separate ultrasonic node).
    """

    def __init__(self) -> None:
        super().__init__("drive_node")

        self.declare_parameter("use_sim", False)
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("wheelbase_m", 0.10)  # PLACEHOLDER — measure axle-to-axle
        self.declare_parameter("max_motor_speed", 50)
        self.declare_parameter("max_steering_angle", 30.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("ultrasonic_rate_hz", 10.0)
        self.declare_parameter("steering_sign", -1)

        self._use_sim = self.get_parameter("use_sim").value
        self._max_linear = self.get_parameter("max_linear_vel").value
        self._wheelbase = self.get_parameter("wheelbase_m").value
        self._max_speed = self.get_parameter("max_motor_speed").value
        self._max_angle = self.get_parameter("max_steering_angle").value
        self._cmd_timeout = self.get_parameter("cmd_timeout").value
        self._ultrasonic_rate = self.get_parameter("ultrasonic_rate_hz").value
        self._steering_sign = self.get_parameter("steering_sign").value

        if not self._use_sim:
            from picarx import Picarx  # type: ignore[import]
            self._px = Picarx()
            self._px.set_cam_tilt_angle(2)
            self._px.set_cam_pan_angle(13)
        else:
            self._px = None

        self._sub = self.create_subscription(Twist, "/rover/cmd_vel", self._on_cmd_vel, 10)
        self._pan_sub = self.create_subscription(Float32, "/rover/camera/pan", self._on_pan, 10)
        self._range_pub = self.create_publisher(
            Range, "/rover/ultrasonic/range", qos_profile_sensor_data
        )
        self._last_cmd = self.get_clock().now()
        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0 / self._ultrasonic_rate, self._publish_range)

        self.get_logger().info(f"Drive node ready (sim={self._use_sim})")

    def _on_pan(self, msg: Float32) -> None:
        if self._px is not None:
            self._px.set_cam_pan_angle(float(msg.data))

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = self.get_clock().now()
        self._drive(msg.linear.x, msg.angular.z)

    def _drive(self, linear_x: float, angular_z: float) -> None:
        speed = int(
            max(-self._max_speed,
                min(self._max_speed,
                    linear_x / self._max_linear * self._max_speed))
        )

        # Ackermann steering angle from curvature (bicycle model), not a
        # flat linear scale of angular_z: the wheel angle needed for a given
        # turn depends on speed too — curvature = angular_z / linear_x, so
        # the same angular_z means a much sharper required turn at low speed
        # than at high speed. Scaling angular_z alone (the old formula)
        # ignored that entirely, badly understeering at Nav2's slow cruise
        # speed and leaving the rover barely tracking curved paths at all.
        if abs(linear_x) < 1e-3:
            angle = 0.0
        else:
            curvature = angular_z / linear_x
            angle = self._steering_sign * math.degrees(math.atan(self._wheelbase * curvature))
            angle = max(-self._max_angle, min(self._max_angle, angle))

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

    def _publish_range(self) -> None:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees in radians
        msg.min_range = 0.02
        msg.max_range = 4.00

        if self._use_sim:
            msg.range = 1.0
        else:
            reading = self._px.ultrasonic.read()
            if reading is None or reading < 0:
                return
            msg.range = float(reading) / 100.0  # cm → m

        self._range_pub.publish(msg)

    def _watchdog(self) -> None:
        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if elapsed > self._cmd_timeout:
            self._drive(0.0, 0.0)

    def destroy_node(self) -> None:
        if self._px is not None:
            try:
                import signal
                signal.alarm(3)
                self._px.stop()
                signal.alarm(0)
            except Exception:
                pass
            self._px = None
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
