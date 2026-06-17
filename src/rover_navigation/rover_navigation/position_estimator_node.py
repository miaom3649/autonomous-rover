import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Float64


class KalmanFilter1D:
    """1D Kalman filter with a single state: position.

    Velocity command is treated as a control input, not a state.
    State equation:  x(t+dt) = x(t) + velocity_cmd * dt
    Observation:     z = x  (H = 1, ultrasonic measures position directly)
    """

    def __init__(self, q_pos: float, r_pos: float) -> None:
        self._x: float = 0.0    # position (m)
        self._P: float = 10.0   # large initial uncertainty

        self._Q: float = q_pos  # process noise variance (m^2)
        self._R: float = r_pos  # measurement noise variance (m^2)

        self._initialized = False

    def initialize(self, position: float) -> None:
        """Seed the filter with the first sensor reading."""
        self._x = position
        self._initialized = True

    def predict(self, velocity_cmd: float, dt: float) -> None:
        """Predict next position using commanded velocity (dead reckoning)."""
        self._x = self._x + velocity_cmd * dt
        self._P = self._P + self._Q

    def update(self, position_measurement: float) -> None:
        """Correct predicted position using ultrasonic distance reading."""
        y = position_measurement - self._x     # innovation
        S = self._P + self._R                  # innovation covariance
        K = self._P / S                        # Kalman gain
        self._x = self._x + K * y
        self._P = (1.0 - K) * self._P

    @property
    def position(self) -> float:
        return self._x

    @property
    def is_initialized(self) -> bool:
        return self._initialized


class PositionEstimatorNode(Node):
    """Fuses ultrasonic range and cmd_vel to estimate 1D position via Kalman filter.

    Subscribes:
        /rover/ultrasonic/range  (sensor_msgs/Range)   — position measurement
        /rover/cmd_vel           (geometry_msgs/Twist)  — dead reckoning velocity

    Publishes:
        /rover/estimated_position  (std_msgs/Float64)  — filtered distance to wall (m)
    """

    def __init__(self) -> None:
        super().__init__("position_estimator_node")

        self.declare_parameter("q_pos", 0.01)   # process noise — position (m^2)
        self.declare_parameter("r_pos", 0.05)   # measurement noise — ultrasonic (m^2)
        self.declare_parameter("predict_rate_hz", 20.0)

        q_pos: float = self.get_parameter("q_pos").value
        r_pos: float = self.get_parameter("r_pos").value
        rate_hz: float = self.get_parameter("predict_rate_hz").value

        self._kf = KalmanFilter1D(q_pos=q_pos, r_pos=r_pos)
        self._latest_cmd_vel: float = 0.0
        self._last_predict_time = self.get_clock().now()

        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._sub_range = self.create_subscription(
            Range, "/rover/ultrasonic/range", self._range_callback, qos_profile_sensor_data
        )
        self._sub_cmd_vel = self.create_subscription(
            Twist, "/rover/cmd_vel", self._cmd_vel_callback, reliable_qos
        )
        self._pub_position = self.create_publisher(Float64, "/rover/estimated_position", 10)

        self._predict_timer = self.create_timer(1.0 / rate_hz, self._predict_step)

        self.get_logger().info(
            f"Position estimator ready — q_pos={q_pos}, r_pos={r_pos}, "
            f"predict_rate={rate_hz} Hz"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        # Rover faces the wall: forward motion (linear.x > 0) decreases distance.
        self._latest_cmd_vel = -msg.linear.x

    def _range_callback(self, msg: Range) -> None:
        distance_m = msg.range
        if not (msg.min_range <= distance_m <= msg.max_range):
            return

        if not self._kf.is_initialized:
            self._kf.initialize(distance_m)
            self.get_logger().info(f"Filter initialised at {distance_m:.3f} m")
            return

        self._kf.update(distance_m)
        self._publish()

    def _predict_step(self) -> None:
        if not self._kf.is_initialized:
            return

        now = self.get_clock().now()
        dt = (now - self._last_predict_time).nanoseconds * 1e-9
        self._last_predict_time = now

        self._kf.predict(self._latest_cmd_vel, dt)
        self._publish()

    def _publish(self) -> None:
        pos_msg = Float64()
        pos_msg.data = self._kf.position
        self._pub_position.publish(pos_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PositionEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
