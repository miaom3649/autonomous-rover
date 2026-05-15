# autonomous-rover
An autonomous robotic vehicle that uses real-time computer vision to detect and avoid obstacles without human intervention.

---

## Raspberry Pi Setup (run after every fresh flash)

Steps to configure the Pi from scratch. SSH in from the dev machine unless noted otherwise.

### 1. Connect to WiFi

Configure WiFi and hostname (`raspberrypi`) via Raspberry Pi Imager before flashing. Ubuntu 22.04 Server includes mDNS support out of the box.

Verify the Pi is reachable from the dev machine:
```bash
ping -4 raspberrypi.local
```

### 2. Configure SSH key authentication

On the **dev machine**, copy your public key to the Pi so `deploy.sh` runs without a password prompt:
```bash
ssh-copy-id konkon@raspberrypi.local
```

### 4. Install ROS2 Humble

SSH into the Pi and run:

```bash
# Locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt source
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install (ros-base, no GUI)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions

# Auto-source on login
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version
```

### 5. Clone the repository

```bash
mkdir -p ~/dev
cd ~/dev
git clone git@github.com:miaom3649/autonomous-rover.git
```

### 6. Verify deploy works

Back on the **dev machine**:
```bash
./scripts/deploy.sh
```

Should pull latest code and run `colcon build` without errors.
