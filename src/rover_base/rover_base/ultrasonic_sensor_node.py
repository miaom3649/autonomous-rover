import math
import random
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


# Robot Hat V4 ultrasonic pin assignments.
_DEFAULT_TRIG_PIN = "D2"
_DEFAULT_ECHO_PIN = "D3"

# Physical limits of the HC-SR04 sensor (metres).
_MIN_RANGE_M = 0.02
_MAX_RANGE_M = 4.00

# Simulated wall distance oscillates between these bounds (metres).
_SIM_CENTER_M = 0.60
_SIM_AMPLITUDE_M = 0.30
_SIM_NOISE_STD_M = 0.02
_SIM_PERIOD_S = 10.0


class UltrasonicSensorNode(LifecycleNode):
    """Lifecycle driver node for the HC-SR04-compatible ultrasonic sensor.

    Publishes sensor_msgs/Range to /rover/ultrasonic/range.
    Set use_sim:=true to run without physical hardware.
    """

    def __init__(self) -> None:
        super().__init__("ultrasonic_sensor_node")
        self._timer = None
        self._sensor = None
        self._sim_start_time: float = 0.0

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.declare_parameter("use_sim", False)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("trig_pin", _DEFAULT_TRIG_PIN)
        self.declare_parameter("echo_pin", _DEFAULT_ECHO_PIN)

        self._use_sim: bool = self.get_parameter("use_sim").value
        self._rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._trig_pin: str = self.get_parameter("trig_pin").value
        self._echo_pin: str = self.get_parameter("echo_pin").value

        # Use a regular publisher — lifecycle state is controlled via timer start/stop.
        # create_lifecycle_publisher in rclpy/Humble does not reliably activate
        # managed publishers automatically on Python lifecycle nodes.
        self._publisher = self.create_publisher(
            Range, "/rover/ultrasonic/range", qos_profile_sensor_data
        )

        if not self._use_sim:
            if not self._init_hardware():
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info(
            f"Configured — mode={'sim' if self._use_sim else 'hardware'}, "
            f"rate={self._rate_hz} Hz"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._sim_start_time = time.monotonic()
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_reading)
        self.get_logger().info("Activated — publishing range data.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self.get_logger().info("Deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._sensor = None
        self.get_logger().info("Cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._sensor = None
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _init_hardware(self) -> bool:
        try:
            from robot_hat import Pin, Ultrasonic  # type: ignore
        except ImportError:
            self.get_logger().error(
                "robot_hat library not found. Run on the Raspberry Pi or use use_sim:=true."
            )
            return False
        try:
            self._sensor = Ultrasonic(Pin(self._trig_pin), Pin(self._echo_pin))
        except Exception as exc:
            self.get_logger().error(f"Failed to initialise ultrasonic sensor: {exc}")
            return False
        return True

    def _publish_reading(self) -> None:
        distance_m = self._read_sim() if self._use_sim else self._read_hardware()
        if distance_m is None:
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees, typical for HC-SR04
        msg.min_range = _MIN_RANGE_M
        msg.max_range = _MAX_RANGE_M
        msg.range = distance_m
        self._publisher.publish(msg)

    def _read_hardware(self) -> float | None:
        try:
            raw_cm = self._sensor.read()
        except Exception as exc:
            self.get_logger().warning(f"Sensor read error: {exc}")
            return None

        if raw_cm is None:
            self.get_logger().debug("No echo received.")
            return None

        distance_m = raw_cm / 100.0
        if not (_MIN_RANGE_M <= distance_m <= _MAX_RANGE_M):
            self.get_logger().debug(f"Out-of-range reading: {distance_m:.3f} m")
            return None

        return distance_m

    def _read_sim(self) -> float:
        elapsed = time.monotonic() - self._sim_start_time
        true_distance = _SIM_CENTER_M + _SIM_AMPLITUDE_M * math.sin(
            2 * math.pi * elapsed / _SIM_PERIOD_S
        )
        noise = random.gauss(0.0, _SIM_NOISE_STD_M)
        return max(_MIN_RANGE_M, true_distance + noise)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
