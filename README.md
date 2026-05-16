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

---

## CSI Camera Setup (Ubuntu 22.04)

Ubuntu 22.04 ships a 2020-era libcamera with no Python bindings. `picamera2` requires them, so they must be compiled from source. This is a one-time step.

**Background:** CSI cameras (Pi Camera Module) output raw Bayer sensor data that must pass through an ISP before becoming a usable image. On Raspberry Pi OS this pipeline is handled automatically; on Ubuntu it requires a manually built libcamera.

### 1. Add swap space

Compiling libcamera is memory-intensive. Without swap the Pi will hang and require a hard reboot.

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### 2. Install build dependencies

```bash
sudo apt install -y git ninja-build pkg-config \
    python3-pybind11 python3-jinja2 python3-ply python3-yaml \
    libudev-dev libgnutls28-dev python3-dev libevent-dev
```

Install a recent meson (Ubuntu 22.04's apt version is too old):

```bash
pip3 install --user 'meson>=1.0.1'
export PATH="$HOME/.local/bin:$PATH"   # add to ~/.bashrc to persist
```

### 3. Build libcamera from source

```bash
cd ~
git clone https://git.libcamera.org/libcamera/libcamera.git
cd libcamera
meson setup build --prefix=/usr/local \
    -Dpycamera=enabled \
    -Dpipelines=rpi/vc4 \
    -Dipas=rpi/vc4
cd build
nice -n 19 ninja -j1   # single-threaded to avoid OOM; takes ~30 min
```

### 4. Install libcamera

`sudo ninja install` will fail because sudo picks up the old system meson. Use:

```bash
sudo pip3 install 'meson>=1.0.1'   # install new meson system-wide for sudo
sudo ninja install
sudo ldconfig
```

Fix the install path (libcamera lands in `python3/` but Python 3.10 looks in `python3.10/`):

```bash
sudo ln -s /usr/local/lib/python3/dist-packages/libcamera \
           /usr/local/lib/python3.10/dist-packages/libcamera
```

Verify:

```bash
python3 -c "import libcamera; print('OK')"
```

### 5. Create pykms stub

`picamera2` imports `pykms` (a display driver library) at load time even when running headless. Since the Pi is used without a monitor, a stub module satisfies the import without needing real KMS/DRM hardware:

```bash
mkdir -p ~/.local/lib/python3.10/site-packages/pykms
```

Create `~/.local/lib/python3.10/site-packages/pykms/__init__.py` with the contents from [scripts/pykms_stub.py](scripts/pykms_stub.py) — or run:

```bash
cp ~/dev/autonomous-rover/scripts/pykms_stub.py \
   ~/.local/lib/python3.10/site-packages/pykms/__init__.py
```

### 6. Verify the camera works

```bash
cd ~/dev/autonomous-rover
python3 scripts/test_camera.py
```

Expected output:
```
=== Camera Diagnostic ===
[media-ctl] CSI sensor detected: - entity 1: ov5647 10-0036 ...
[picamera2] Captured frame saved to: .../camera_test.jpg
Result: PASS (CSI / Pi Camera via picamera2)
```

The captured JPEG is saved to `/tmp/camera_test.jpg` by default (use `--output` to override).

---

### Troubleshooting

**`meson setup` fails with `requires >= 1.0.1`**
The apt meson is too old. Install via pip and re-export PATH (step 2).

**`ninja -j4` hangs and SSH drops**
Out of memory. Add swap (step 1) and use `ninja -j1`.

**`sudo ninja install` fails with `No module named 'mesonbuild.options'`**
Two meson versions conflict. Run `sudo pip3 install 'meson>=1.0.1'` then retry.

**`import libcamera` fails after install**
The `.so` is in the wrong path. Run the `ln -s` symlink command in step 4.

**`picamera2` import fails with `No module named 'pykms'`**
The pykms stub is missing. Follow step 5.

**`picamera2` import fails with `cannot import name 'PixelFormat' from 'pykms'`**
The pykms stub is incomplete. Replace it with the version from `scripts/pykms_stub.py`.
