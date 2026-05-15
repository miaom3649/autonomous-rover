# CLAUDE.md — Autonomous Rover Project

## Project Overview

An autonomous rover built on **Raspberry Pi 4** with **Sunfounder Robot Hat V4** as the motor/servo driver board. The rover uses ROS2 as its middleware and supports AI-based obstacle avoidance, SLAM mapping, autonomous navigation, and manual/auto mode switching.

**Key hardware:**
- Main computer: Raspberry Pi 4
- Driver board: Sunfounder Robot Hat V4
- Sensors: RGB camera (Pi Camera or USB), ultrasonic distance sensor (HC-SR04 compatible)

---

## ROS2 Workspace Layout

Follow standard ROS2 workspace conventions:

```
autonomous-rover/
├── src/                    # All ROS2 packages live here
│   ├── rover_bringup/      # Launch files and top-level configs
│   ├── rover_base/         # Hardware abstraction (motor, ultrasonic, camera)
│   ├── rover_perception/   # Obstacle detection, camera processing
│   ├── rover_navigation/   # Path planning, SLAM integration
│   └── rover_control/      # Mode switching, manual/auto controller
├── config/                 # Shared YAML parameter files
├── scripts/                # Utility scripts (setup, deploy, hardware diagnostics)
├── tests/                  # Non-ROS unit tests (pytest)
├── CLAUDE.md
└── README.md
```

Each ROS2 package must contain a `package.xml` and `CMakeLists.txt` (C++) or `setup.py` (Python).

---

## Language and Style

### Python
- Version: Python 3.10+
- Style: **PEP8**, enforced by `black` (line length 100) and `flake8`
- Type hints are required on all public functions and class methods
- Use `rclpy` for all ROS2 Python nodes
- Docstrings: Google Style, one-line for simple functions, full format for public API

### C++
- Standard: C++17
- Style: [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use `ament_lint_auto` and `ament_cmake_cpplint` for linting
- Prefer `std::shared_ptr` and RAII over raw pointers

### General
- All identifiers and comments must be in **English**
- No magic numbers — define named constants or use ROS2 parameters
- Avoid global mutable state; prefer dependency injection via constructor

---

## ROS2 Node Conventions

- One node per file; filename matches the node name (e.g., `ultrasonic_sensor_node.py`)
- Node names use `snake_case`; topic/service names use `snake_case` with a leading namespace, e.g., `/rover/cmd_vel`
- Always declare parameters with `declare_parameter()` and load from YAML; never hardcode values
- Use QoS profiles explicitly — prefer `SensorDataQoS` for sensor topics, `ReliableQoS` for control commands
- Lifecycle nodes (`rclpy.lifecycle`) are preferred for hardware driver nodes so bringup/teardown is controllable

---

## Hardware Abstraction Rules

- All Sunfounder Robot Hat V4 interactions must go through dedicated driver nodes in `rover_base`
- No other package may import or call hardware libraries (e.g., `picar-x`, GPIO, SMBus) directly — always go through ROS2 topics/services
- Hardware driver nodes must expose a mock/sim mode toggled by a ROS2 parameter `use_sim:=true` so unit tests can run without physical hardware

---

## Testing Strategy

Testing is **unit tests first, hardware second**. Never skip to hardware testing without passing unit tests.

### Unit Tests
- Framework: `pytest` for Python, `gtest` for C++
- All business logic (perception, planning, control decisions) must have unit tests
- Hardware driver nodes must be testable in sim mode (`use_sim:=true`)
- Test files live in `tests/` at the repo root (non-ROS) or in `<package>/test/` (ROS integration tests)
- Run before every commit: `colcon test --packages-select <package>`

### Hardware Tests
- Only run after all unit tests pass
- Hardware tests are tagged `@pytest.mark.hardware` and excluded from CI
- Document expected sensor ranges and failure modes in the test file header

### CI (GitHub Actions)

CI runs on every push and pull request targeting `main`. The workflow file lives at `.github/workflows/ci.yml`.

**Stages (run in order):**

| Stage | Tool | Command |
|-------|------|---------|
| Lint — Python | `black`, `flake8` | `black --check --line-length 100 src/ tests/` then `flake8 --max-line-length 100 src/ tests/` |
| Lint — C++ | `ament_lint_auto` | `colcon test --packages-select <pkg> --pytest-args -k ament_lint` |
| Build | `colcon` | `colcon build --symlink-install` |
| Test (sim) | `colcon` / `pytest` | `colcon test --event-handlers console_direct+ -- --ros-args -p use_sim:=true` |

**Rules:**
- All stages must pass before a PR can be merged
- Hardware-tagged tests (`@pytest.mark.hardware`) are excluded from CI via `pytest -m "not hardware"`
- CI runs in sim mode only — no physical hardware access
- A failing lint stage blocks the build and test stages from running

---

## Mode Switching

The rover supports two operating modes:

| Mode | Trigger | Behavior |
|------|---------|----------|
| `MANUAL` | Operator command via topic `/rover/mode` | Follows `/rover/cmd_vel` from teleop |
| `AUTO` | Operator command or automatic trigger | Runs autonomous navigation stack |

- Mode transitions must be logged at `INFO` level
- AUTO mode must yield to MANUAL within 100 ms of receiving a mode change command
- Emergency stop (`/rover/estop`) overrides both modes and cuts motor output immediately

---

## Sensor Topics (Planned)

| Topic | Message Type | Source Node |
|-------|-------------|-------------|
| `/rover/camera/image_raw` | `sensor_msgs/Image` | camera driver |
| `/rover/ultrasonic/range` | `sensor_msgs/Range` | ultrasonic driver |
| `/rover/odom` | `nav_msgs/Odometry` | base driver |
| `/rover/cmd_vel` | `geometry_msgs/Twist` | navigation / teleop |

---

## Scripts Directory

`scripts/` contains two types of scripts:

**Dev/ops scripts** — environment setup and deployment:
- `setup.sh` — install dependencies and configure the environment
- `deploy.sh` — SSH into the Raspberry Pi, pull latest code, and rebuild

**Hardware diagnostic scripts** — run directly on physical hardware to verify each component works before bringing up ROS:
```
scripts/
├── test_camera.py        # Capture a photo and save locally
├── test_motor.py         # Drive forward 2s then stop
├── test_ultrasonic.py    # Print distance readings continuously
└── test_servo.py         # Sweep servo left, center, right
```

These diagnostic scripts access the Sunfounder SDK directly (bypassing ROS and `rover_base`) — this is intentional. Each script must include the following header comment:

```python
# Hardware diagnostic script — direct SDK access intentional.
# Not a ROS node. Run on physical hardware only.
```

---

## Python Virtual Environment (Dev Machine Only)

Python toolchain tools (linters, formatters, test runners) must be installed inside a `venv` on the development machine to avoid polluting the system Python.

**Setup:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-dev.txt
```

**`requirements-dev.txt` must pin exact versions**, e.g.:
```
black==24.4.2
flake8==7.0.0
pytest==8.2.0
```

**Rules:**
- `.venv/` is gitignored — never commit it
- Always activate the venv before running `black`, `flake8`, or `pytest` locally
- Do not install `rclpy` or any ROS2 packages into the venv — ROS2 is sourced separately via `source /opt/ros/humble/setup.bash`
- On the Raspberry Pi, Python tools are installed system-wide via `apt`; no venv is needed there

---

## Development Workflow

Development is done on a separate machine; the Raspberry Pi only runs code. The two are connected via **local WiFi (LAN)**.

**Standard flow for every code change:**

```
1. Write code on the dev machine
2. git push to GitHub
3. Run ./scripts/deploy.sh — this SSHes into the Pi, pulls latest code, and rebuilds
4. Start ROS on the Pi manually when ready to test
```

`deploy.sh` connects via:
```bash
ROVER_HOST="raspberrypi.local"   # mDNS hostname — works regardless of IP changes
ROVER_USER="konkon"
```

SSH key-based auth must be configured (add dev machine's public key to `~/.ssh/authorized_keys` on the Pi) so deploy.sh runs without a password prompt.

> **Tip:** To avoid chasing a changing IP, assign the Pi a static IP in your router's DHCP settings (bind by MAC address), or enable mDNS on the Pi (`sudo apt install avahi-daemon`) and connect via `rover.local` instead of an IP.

**Do not develop directly on the Raspberry Pi.**

---

## Commit and Branch Conventions

- Branch naming: `feat/<topic>`, `fix/<topic>`, `chore/<topic>`
- Commits: imperative mood, under 72 chars, e.g. `add ultrasonic driver node`
- Do not commit generated files (`build/`, `install/`, `log/`, `__pycache__/`)
- `.gitignore` must cover ROS2 build artifacts, Python cache, and the dev venv: `build/`, `install/`, `log/`, `__pycache__/`, `.venv/`

---

## What Claude Should Know

- **Do not** access GPIO, I2C, SPI, or the Sunfounder SDK directly from any non-`rover_base` package
- **Always** check if a ROS2 parameter is declared before reading it
- **Prefer** composition over inheritance for ROS2 nodes
- **Do not** add `print()` for debugging — use `self.get_logger().debug()`
- When adding a new sensor or actuator, add it to the Sensor Topics table above and create a corresponding driver node in `rover_base`
- When in doubt about hardware behavior, implement sim mode first and validate logic before touching real hardware
