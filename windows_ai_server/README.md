# Windows AI server

GPU inference runs on Windows so the Raspberry Pi can keep its CPU and memory
available for ROS, lidar odometry, SLAM, and Nav2.

## Install

Open PowerShell in this directory. An existing virtual environment can be used
without activating it (useful when PowerShell blocks `Activate.ps1`):

```powershell
..\venv\Scripts\python.exe -m pip install --upgrade pip
..\venv\Scripts\python.exe -m pip install -r requirements.txt
```

PyTorch is intentionally not pinned in `requirements.txt`, because its install
command depends on the NVIDIA driver/CUDA version. If the environment does not
already have CUDA-enabled PyTorch, install the appropriate build from the
[PyTorch installer](https://pytorch.org/get-started/locally/) first. Verify it:

```powershell
..\venv\Scripts\python.exe -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

## YOLO object detection

```powershell
..\venv\Scripts\python.exe object_server.py
```

The first run downloads `yolo11n.pt`. The service listens on port `8766`:

```text
GET  /health
POST /detect
```

Allow inbound TCP port 8766 in Windows Firewall, then put the Windows LAN IP in
`config/object_detection_params.yaml` on the rover project.

## Metric depth (optional/experimental)

```powershell
..\venv\Scripts\python.exe depth_server.py
```

This service listens on port `8765` and downloads `Intel/zoedepth-nyu` on its
first run. It is currently used by `scripts/depth_viewer.py` for diagnostics,
not by the main lidar navigation launch.
