import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Float64


class KalmanFilter1D:
    """1D Kalman filter with states [position, velocity].

    Dead reckoning is provided by an external velocity command (cmd_vel).
    Position is observed directly via the ultrasonic sensor (H = [1, 0]).
    """

    def __init__(self, q_pos: float, q_vel: float, r_pos: float) -> None:
        self._x = np.zeros(2)           # [position (m), velocity (m/s)]
        self._P = np.eye(2) * 10.0      # large initial uncertainty

        # Process noise: how much we distrust the motion prediction.
        self._Q = np.diag([q_pos, q_vel])

        # Measurement noise: variance of the ultrasonic sensor (m^2).
        self._R = np.array([[r_pos]])

        # Observation matrix: we measure position directly.
        self._H = np.array([[1.0, 0.0]])

        # State transition: velocity is set directly from cmd_vel (no memory).
        self._F = np.array([[1.0, 0.0], [0.0, 0.0]])

        self._initialized = False

    def initialize(self, position: float) -> None:
        """Seed the filter with the first sensor reading."""
        self._x[0] = position
        self._x[1] = 0.0
        self._initialized = True

    def predict(self, velocity_cmd: float, dt: float) -> None:
        """Predict next state using commanded velocity (dead reckoning).

        position_new = position + velocity_cmd * dt
        velocity_new = velocity_cmd
        """
        B = np.array([dt, 1.0])
        self._x = self._F @ self._x + B * velocity_cmd
        self._P = self._F @ self._P @ self._F.T + self._Q

    def update(self, position_measurement: float) -> None:
        """Correct predicted state using ultrasonic distance reading."""
        z = np.array([position_measurement])
        y = z - self._H @ self._x                          # innovation
        S = self._H @ self._P @ self._H.T + self._R        # innovation covariance
        K = self._P @ self._H.T @ np.linalg.inv(S)         # Kalman gain
        self._x = self._x + K.flatten() * y[0]
        self._P = (np.eye(2) - K @ self._H) @ self._P

    @property
    def position(self) -> float:
        return float(self._x[0])

    @property
    def velocity(self) -> float:
        return float(self._x[1])

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
        /rover/estimated_velocity  (std_msgs/Float64)  — estimated velocity (m/s)
    """

    def __init__(self) -> None:
        super().__init__("position_estimator_node")

        self.declare_parameter("q_pos", 0.01)   # process noise — position (m^2)
        self.declare_parameter("q_vel", 0.10)   # process noise — velocity (m^2/s^2)
        self.declare_parameter("r_pos", 0.05)   # measurement noise — ultrasonic (m^2)
        self.declare_parameter("predict_rate_hz", 20.0)

        q_pos: float = self.get_parameter("q_pos").value
        q_vel: float = self.get_parameter("q_vel").value
        r_pos: float = self.get_parameter("r_pos").value
        rate_hz: float = self.get_parameter("predict_rate_hz").value

        self._kf = KalmanFilter1D(q_pos=q_pos, q_vel=q_vel, r_pos=r_pos)
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
        self._pub_velocity = self.create_publisher(Float64, "/rover/estimated_velocity", 10)

        self._predict_timer = self.create_timer(1.0 / rate_hz, self._predict_step)

        self.get_logger().info(
            f"Position estimator ready — q_pos={q_pos}, q_vel={q_vel}, "
            f"r_pos={r_pos}, predict_rate={rate_hz} Hz"
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

        vel_msg = Float64()
        vel_msg.data = self._kf.velocity
        self._pub_velocity.publish(vel_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PositionEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
