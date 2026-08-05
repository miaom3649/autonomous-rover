# autonomous-rover
An autonomous robotic vehicle that uses real-time computer vision to detect and avoid obstacles without human intervention.

## Camera ground-obstacle marking

Development and non-ROS unit checks run in the VM. ROS integration and
hardware checks run only after `scripts/deploy.sh` pushes the current branch,
updates the Raspberry Pi workspace, and builds it there.

The navigation dashboard can project a clicked camera pixel onto the flat
ground and mark that point on the current SLAM map. First calibrate the camera:

```bash
python3 scripts/capture_calibration_frames.py
python3 scripts/run_camera_calibration.py
```

Calibration writes the intrinsics to `config/camera_projection_params.yaml`.
Measure the physical camera mount and fill in `camera_height_m`, `camera_x_m`,
`camera_y_m`, `camera_pitch_down_deg`, and `camera_yaw_left_deg`; projection is
intentionally disabled while the height remains zero. Keep the pan/tilt fixed
at those measured angles. Start navigation, open
`http://raspberrypi.local:8082`, and click the point where an obstacle touches
the ground in the camera image. The dashboard logs its `base_link` and `map`
coordinates and draws an orange marker. Resetting the SLAM map also clears all
camera obstacle markers.

### Automatic object detection

YOLO inference runs on the Windows GPU rather than the Raspberry Pi. In a
Windows virtual environment, install `scripts/windows_depth_server/requirements.txt`
and start:

```powershell
python scripts/windows_depth_server/object_server.py
```

The first run downloads the small `yolo11n.pt` model. Set `server_url` in
`config/object_detection_params.yaml` to the Windows LAN address, for example
`http://192.168.3.100:8766/detect`, then deploy and start navigation normally.
The camera view shows every YOLO box. Only classes listed in `ground_labels`
are projected from the box's bottom centre onto the map, because the flat-ground
projection is invalid for objects resting on tables or shelves. Nearby repeated
detections of the same class are merged into one semantic marker.

---

## Before you start: power

The Raspberry Pi 4 needs a stable **5.1V / 3A (15W)** supply. During bring-up (flashing, installing dependencies, compiling), power the Pi from a dedicated USB-C supply — **not** the Robot Hat's motor battery.

Why this matters: the Robot Hat V4 runs the Pi and the drive motors off the *same* battery through one DC-DC converter. A stalled/blocked motor can draw several amps, sagging the shared rail below the Pi's brownout threshold — the Pi loses power outright (all LEDs dark), not just a software crash. This looks identical to a corrupted SD card or a hung build, and wastes hours misdiagnosing the wrong thing. Check `vcgencmd get_throttled` any time you suspect this — a nonzero low bit means undervoltage is happening *right now*.

Only wire the motor battery back in once you're actually testing driving, and make sure no wheel is blocked/stalled before doing so.

---

## Raspberry Pi Setup (run after every fresh flash)

Steps to configure the Pi from scratch. SSH in from the dev machine unless noted otherwise. All of the build steps below (ROS2, Pangolin, ORB_SLAM3, libcamera) are memory- and time-intensive on a Pi 4 — read [Resource-constrained builds](#resource-constrained-builds-read-this-first) before starting any of them.

### 1. Connect to WiFi

Configure WiFi and hostname (`raspberrypi`) via Raspberry Pi Imager before flashing. Ubuntu 22.04 Server includes mDNS support out of the box.

Verify the Pi is reachable from the dev machine:
```bash
ping -4 raspberrypi.local
```

If `raspberrypi.local` doesn't resolve but the Pi is definitely up, mDNS on the dev machine may be flaky — restart avahi (`sudo systemctl restart avahi-daemon`) or fall back to scanning for the Pi's IP directly (`sudo nmap -sn <subnet>/24`) and use that IP instead of the hostname for the rest of this guide.

### 2. Configure SSH key authentication

On the **dev machine**, copy your public key to the Pi so `deploy.sh` runs without a password prompt:
```bash
ssh-copy-id konkon@raspberrypi.local
```

Also enable lingering for the Pi's user, so `nohup`/`systemd-run --user` background jobs (used throughout the build steps below) survive after the SSH session ends:
```bash
ssh konkon@raspberrypi.local 'sudo loginctl enable-linger konkon'
```

### 3. Enable SSH on the dev machine (required for camera diagnostics)

`test_camera.py` scps captured images back to `log/` on the dev machine over the same SSH session. For this to work, the dev machine must accept incoming SSH connections from the Pi.

On the **dev machine**:
```bash
# Install and start the SSH server if not already running
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
```

Then, on the **Pi**, authorise it to connect back to the dev machine (one-time):
```bash
# Generate a key pair on the Pi if one doesn't exist yet
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519

# Copy the Pi's public key to the dev machine (prompts for dev machine password once)
ssh-copy-id konkon@<dev-machine-ip>
```

After this, `test_camera.py` will automatically copy the captured image to `~/dev/autonomous-rover/log/` on the dev machine each time it runs. The `log/` directory is gitignored and holds all transient diagnostic output.

### 4. Add swap and disable automatic updates

Every build step below needs headroom beyond the Pi's 1.8GB RAM, and none of them can tolerate `apt`/`dpkg` being locked by an unattended background job mid-build.

```bash
# 8GB swap file (2GB is the bare minimum for libcamera alone; ORB_SLAM3 wants more)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Stop unattended-upgrades from grabbing the dpkg lock mid-build
sudo systemctl disable --now apt-daily.timer apt-daily-upgrade.timer unattended-upgrades.service
sudo systemctl mask unattended-upgrades.service
```

### 5. Install ROS2 Humble

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

# Install (ros-base + Nav2, no GUI)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base ros-humble-nav2-bringup ros-humble-navigation2 \
    python3-colcon-common-extensions python3-rosdep git

sudo rosdep init
rosdep update

# Auto-source on login
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 -h
```

**If `apt update`/`apt install` times out reaching `ports.ubuntu.com` or `packages.ros.org`:** some networks route IPv6 to these mirrors through a dead path, and `apt` wastes its retry budget on IPv6 addresses before giving up. Force IPv4:
```bash
echo 'Acquire::ForceIPv4 "true";' | sudo tee /etc/apt/apt.conf.d/99force-ipv4
```
If you're behind a firewall that blocks these domains outright, relay through a proxy on your dev machine (`Acquire::http::Proxy "http://<dev-machine-ip>:<port>";` in `/etc/apt/apt.conf.d/95proxy`, plus matching `http_proxy`/`https_proxy` in `/etc/environment` and `git config --global http.proxy`) — but test each domain individually first (`curl -m8 -o /dev/null -w '%{http_code}\n' http://ports.ubuntu.com`), since some mirrors work better direct and get *worse* through a proxy.

### 6. Clone the repository

```bash
mkdir -p ~/dev
cd ~/dev
git clone https://github.com/miaom3649/autonomous-rover.git
cd autonomous-rover
rosdep install --from-paths src --ignore-src -r -y
```

Use the `https://` URL, not `git@github.com:...` — the Pi has no GitHub SSH key configured, and the repo is public, so HTTPS just works.

### 7. Verify deploy works

Back on the **dev machine**:
```bash
./scripts/deploy.sh
```

Should pull latest code and run `colcon build` without errors (once the dependency sections below are done).

---

## Resource-constrained builds: read this first

Pangolin, ORB_SLAM3, and libcamera all involve compiling C++ on a Pi 4 with 1.8GB RAM. Treat every one of them with the same defensive pattern, learned the hard way from repeated build-time crashes and one SD card scare:

- **Always `-j1`.** Parallel compilation multiplies peak memory per translation unit. `-j1` is slower but the only setting that reliably avoids OOM on this hardware.
- **Swap must be in place first** (see step 4 above). Without it, an out-of-memory compiler process can take the whole system down instead of just failing.
- **Detach fully from the SSH session**, so a dropped connection or a closed laptop lid doesn't kill a multi-hour build:
  ```bash
  nohup setsid bash -c '... your build commands ...' > ~/build.log 2>&1 < /dev/null & disown
  ```
  Verify it detached: `ps -o pid,ppid,tty,stat,cmd -p <pid>` should show `PPID=1` and `TT=?`.
- **Cap the build's memory with a cgroup** if you want a hung/runaway compile to die cleanly instead of thrashing the whole system into unresponsiveness:
  ```bash
  systemd-run --scope --user -p MemoryMax=1200M -p MemorySwapMax=6G -- <your build command>
  ```
  This requires `loginctl enable-linger` (step 2) so the scope survives past the SSH session. Without it the whole cgroup gets SIGTERM'd the moment you log out.
- **Don't poll the log every few seconds.** Use a single blocking wait keyed on a completion marker instead:
  ```bash
  until ssh pi 'grep -q MY_DONE_MARKER ~/build.log || grep -qE "^E: " ~/build.log'; do sleep 25; done
  ```

**If SSH stops responding mid-build but `ping` still works:** this is very likely the automatic `fsck` that runs after an unclean shutdown, not a hung/corrupted system. Networking comes up early in boot, independent of the root filesystem check; `sshd` can't start until that check finishes, and on a slow SD card a full check of a large ext4 partition can take a long time. **Waiting it out is almost always faster than power-cycling** — each power cycle just restarts the check from scratch. Only suspect real corruption if the exact same stall reproduces across *multiple independent clean boots*; even then, check the logs (below) before assuming the worst.

**If you do need to inspect what happened after a crash:** pull the SD card, mount it read-only on another Linux machine, and check the journal for the boot in question:
```bash
udisksctl mount -b /dev/sdXN -o ro
journalctl --directory=/media/.../var/log/journal --list-boots
journalctl --directory=/media/.../var/log/journal -b -1 -n 100   # last 100 lines of that boot
```
A clean, regular log (e.g. hourly cron firing right up to the last line, no OOM/`EXT4-fs error` entries) that just *stops* mid-cycle with no warning points to a sudden power loss, not filesystem corruption — `e2fsck -f` will usually come back clean. `grep -riE "EXT4-fs error|corrupt|I/O error" ` across `/var/log/{kern.log,syslog,dmesg*}` is the fastest way to check for genuine damage before assuming a reflash is needed.

---

## ORB_SLAM3 Setup (position source for Nav2)

`rover_slam/orb_slam3_node` is a hard dependency of `nav.launch.py` — it's the only position source, there's no way to skip it. It needs Pangolin (for its viewer) and ORB_SLAM3 itself, both built from source. Read [Resource-constrained builds](#resource-constrained-builds-read-this-first) first; both of these are multi-hour, `-j1`, memory-capped builds.

### 1. Build Pangolin

Ubuntu 22.04's GCC 11 needs one compatibility patch before Pangolin (pinned to v0.6 for ORB_SLAM3 compatibility) will compile:

```bash
sudo apt install -y cmake g++ libgl1-mesa-dev libglew-dev ninja-build \
    libjpeg-dev libpng-dev python3-dev python3-pip \
    libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libepoxy-dev

cd ~
git clone --branch v0.6 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# GCC 11 no longer transitively includes <limits> — patch it in
sed -i '30a #include <limits>' include/pangolin/gl/colour.h

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1   # single-threaded; see "Resource-constrained builds" above

sudo make install
sudo ldconfig
```

`sudo make install` (not just `make`) matters — `rover_slam`'s `find_package(Pangolin REQUIRED)` looks in the standard system paths (`/usr/local`), not the build directory.

### 2. Build ORB_SLAM3

```bash
sudo apt install -y libopencv-dev libboost-all-dev libssl-dev

cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
```

The upstream `build.sh` defaults to `make -j4`/`make -j$(nproc)` and `mkdir build` (fails if the dir exists on a re-run) — fix both before running anything:
```bash
sed -i 's/make -j4/make -j1/g; s/make -j$/make -j1/g; s/make -j\$(nproc)/make -j1/g' build.sh
sed -i 's/^mkdir build$/mkdir -p build/' build.sh
```

Apply two upstream bug fixes **before** building (patches this project needs; not present upstream):
- `src/System.cc`: an absolute atlas save path gets `./` prepended, producing a broken path like `.//home/.../room.osa`.
- `src/Map.cc`: `PreSave()` iterates `mspMapPoints` while `EraseObservation()` can mutate it mid-loop, invalidating the iterator and crashing with `SIGSEGV` in `_Rb_tree_increment`.

Run `scripts/patch_orbslam3.py` for the exact patch logic (it also works after-the-fact on an existing build if you hit this crash later), or apply the same edits manually before the first build to skip a redundant rebuild.

Also worth doing before building: cap `-O3` down to `-O1` on the four heaviest translation units, so the compiler itself doesn't spike memory on this hardware. Insert into `CMakeLists.txt` right before `add_library(${PROJECT_NAME} SHARED`:
```cmake
set_source_files_properties(
  src/Optimizer.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  PROPERTIES COMPILE_OPTIONS "-O1"
)
```

Now build. `rover_slam` only needs `libORB_SLAM3.so` plus the DBoW2/g2o Thirdparty libs — skip ORB_SLAM3's `Examples/` demo binaries entirely by building the `ORB_SLAM3` target directly instead of running the upstream `build.sh` wholesale:
```bash
cd Thirdparty/DBoW2 && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j1
cd ~/ORB_SLAM3/Thirdparty/g2o && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j1
cd ~/ORB_SLAM3/Vocabulary && tar -xf ORBvoc.txt.tar.gz
cd ~/ORB_SLAM3 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1 ORB_SLAM3   # not bare `make` — that also builds ~30 unused Example binaries
```

Recommended: wrap the whole thing in a memory-capped cgroup (see "Resource-constrained builds"), e.g. `systemd-run --scope --user -p MemoryMax=1200M -p MemorySwapMax=6G -- bash build_script.sh`.

Verify:
```bash
file ~/ORB_SLAM3/lib/libORB_SLAM3.so   # should say "ELF 64-bit ... aarch64 ... dynamically linked"
ldd ~/ORB_SLAM3/lib/libORB_SLAM3.so | grep "not found"   # no output = all deps resolved
```

`rover_slam/CMakeLists.txt` requires `ORB_SLAM3_ROOT_DIR` to be set (fatal CMake error otherwise):
```bash
echo 'export ORB_SLAM3_ROOT_DIR=$HOME/ORB_SLAM3' >> ~/.bashrc
```

---

## Sunfounder Robot Hat V4 SDK

`rover_base/drive_node.py` imports `picarx` and `robot_hat`. Neither is a ROS2 package — install them as regular Python packages.

**Do not `pip install robot-hat`.** The `robot-hat` name on PyPI belongs to an unrelated third-party project (a different author, a different API — it's missing `ADC`, `Servo`, `fileDB`, and other classes `picarx` expects). Installing it produces confusing `ImportError`s that look like a version mismatch but are actually a completely different library. Install Sunfounder's real SDK from their GitHub repo instead:

```bash
sudo apt install -y i2c-tools python3-smbus python3-pip portaudio19-dev python3-pyaudio

# picarx has no PyPI package either — install straight from GitHub
pip3 install git+https://github.com/sunfounder/picar-x.git

# robot_hat: use Sunfounder's repo, NOT `pip install robot-hat`
cd ~
git clone https://github.com/sunfounder/robot-hat.git
cd robot-hat
pip3 install .   # not their install.py — it assumes Raspberry Pi OS-specific tooling that doesn't exist on Ubuntu

# transitive deps robot_hat needs that pip won't pull in automatically
pip3 install lgpio smbus2
```

Verify:
```bash
python3 -c "from picarx import Picarx; from robot_hat import Motor, PWM, Pin, Ultrasonic; import smbus2; print('OK')"
```

---

## CSI Camera Setup (Ubuntu 22.04)

Ubuntu 22.04 ships a 2020-era libcamera with no Python bindings. `picamera2` requires them, so they must be compiled from source. This is a one-time step.

**Background:** CSI cameras (Pi Camera Module) output raw Bayer sensor data that must pass through an ISP before becoming a usable image. On Raspberry Pi OS this pipeline is handled automatically; on Ubuntu it requires a manually built libcamera.

**`rover_camera`'s C++ node needs this too — do not `apt install libcamera-dev` as a shortcut.** Ubuntu 22.04's packaged libcamera (`0~git20200629`) is a pre-1.0 API: no `Signal::connect(lambda)`, no argument-less `Camera::start()`, no `FrameBuffer::Plane::offset`, no `Request::reuse()`. `rover_camera/src/camera_node.cpp` is written against current libcamera and won't compile against the apt package — you'll get `pkg_check_modules` or C++ overload-resolution errors that look like environment problems but are actually just the wrong libcamera. The apt package's `.pc` file is also misnamed (`camera.pc` advertising `Name: libcamera`, so `pkg-config libcamera` can't find it even if installed) — don't paper over that with a symlink either; build the real thing below instead, which installs to `/usr/local` and takes priority over anything from apt.

If step 1 (swap) was already done above in the main setup, skip to step 2.

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

**`rover_camera` fails to build with `pkg_check_modules ... A required package was not found`**
Either libcamera isn't built yet (do steps 1–4 above), or apt's `libcamera-dev` got installed at some point and is shadowing/conflicting with the real one. `sudo apt remove libcamera-dev libcamera0` and make sure `/usr/local/lib/pkgconfig/libcamera.pc` exists (from `sudo ninja install`, step 4) before retrying.

**`rover_camera` fails to build with C++ errors like `no matching function for call to 'Signal::connect(...)'` or `'Camera' has no member named 'start'` taking arguments**
Same root cause as above — the compiler is picking up the old apt libcamera headers (`/usr/include/libcamera`) instead of the one at `/usr/local/include/libcamera`. Remove the apt package; don't try to patch `camera_node.cpp` to match the old API, since that's a step backwards from the upstream API the code is intentionally written against.

---

## General Troubleshooting

**`ssh` to the Pi hangs on `kex_exchange_identification` / times out during banner exchange, but `ping` works fine**
See [Resource-constrained builds](#resource-constrained-builds-read-this-first) — almost always the post-crash `fsck`, not a dead system. Wait it out rather than power-cycling again.

**Pi's green (activity) LED goes dark and stays dark, SSH won't connect, but the red (power) LED is solid**
Not a power problem (steady red = clean supply). Check `vcgencmd get_throttled` once you're back in — `0x0` means no undervoltage recorded, which rules out the shared-battery brownout failure mode and points back to the `fsck`/slow-SD-card explanation above.

**Repeated unexplained reboots/power loss, especially correlated with the rover's wheels trying to move**
Check whether a motor is stalled/blocked and drawing current — see "Before you start: power" at the top of this doc. `vcgencmd get_throttled` with bit 0 set means undervoltage *right now*; historical-only bits (16, 18, ...) with the low bits clear mean it happened but isn't currently happening.

**`sudo apt update` fails with a GPG `NO_PUBKEY` error for the ROS2 repo**
The keyring fetch (`ros.key`) happened before network/proxy access was actually working, so a truncated error page got saved as the key file instead of a real one. Check with `file /usr/share/keyrings/ros-archive-keyring.gpg` — it should say "OpenPGP Public Key", not "ASCII text". Delete and refetch once network access is confirmed working.

**`apt install`/`apt update` hangs on `Waiting for cache lock: ... held by process ... (unattended-upgr)`**
The distro's own background updater grabbed the dpkg lock. It's a real (if annoying) upgrade, not stuck — wait for it to finish, then follow step 4 in the main setup above to disable it permanently so this doesn't recur mid-build.

**A long-running background install/build seems to have silently "completed" way too fast**
Check for accidental double-backgrounding: if a command is both launched with a tool's own background flag *and* internally does `... & disown` itself, the outer wrapper returns almost immediately (as soon as the inner job is launched) while the real work is still running detached and untracked. Prefer one layer of backgrounding, not two — either the tool's own async mechanism, or a manual `nohup ... & disown`, not both stacked on the same command.

**A wait-loop for a background process exits immediately even though the process is clearly still running**
If the loop's exit condition uses `pgrep -x <name>`, check whether `<name>` is longer than 15 characters — Linux truncates `/proc/<pid>/comm` to `TASK_COMM_LEN` (15 chars), so `pgrep -x` against a longer name never matches. Use `pgrep -f <name>` (matches the full command line instead) when the process/marker name might be long.
