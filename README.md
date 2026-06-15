# mercury

Official repository for ICMTC UGVC-2026

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- colcon
- rosdep
- Docker (optional)

---

## First-Time Setup (Fresh Clone)

This repository is already a ROS 2 workspace.

```bash
# Clone workspace
git clone <repo-url>
cd mercury

# Source ROS
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
pip install opencv-python numpy psutil --break-system-packages

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

---

## Python Virtual Environment Setup

Some packages (e.g., turret vision) require Python dependencies that must be installed in a virtual environment alongside ROS 2.

```bash
# Create venv — allow access to ROS 2 system packages
python3 -m venv ~/mercury_venv --system-site-packages

# Activate
source ~/mercury_venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

Add activation to your `~/.bashrc` so it persists across terminals:

```bash
echo "source ~/mercury_venv/bin/activate" >> ~/.bashrc
source ~/.bashrc
```

> **Note:** Always activate the venv before running any turret vision or perception nodes. The `--system-site-packages` flag ensures ROS 2 Python packages (`rclpy`, etc.) remain accessible inside the venv.

---

## Environment Setup

Add this to your `~/.bashrc` or `~/.zshrc`:

```bash
# ROS
source /opt/ros/jazzy/setup.bash

# Workspace
source ~/mercury/install/setup.bash

# Gazebo resource path
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix simulation)/share/simulation/models:$GZ_SIM_RESOURCE_PATH

# Gazebo system plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib

# Python venv
source ~/mercury_venv/bin/activate
```

Apply:

```bash
source ~/.bashrc
```

---

## Running with Docker

```bash
sudo docker compose build
sudo docker compose run ros
```

---

## Running Simulation

```bash
cd mercury
source install/setup.bash
colcon build
ros2 launch bringup bringup_sim.launch.py
```

---

## Monitoring Stack (`watchdog_monitor`)

A non-intrusive ROS 2 monitoring and observability package for the Mercury robot.

### Nodes

| Node | Publishes | Rate | Description |
|------|-----------|------|-------------|
| `health` | `/system_status`, `/system_alerts` | 2s | Merged system monitor + watchdog (tracks nodes, topics, TF) |
| `waypoints` | `/waypoint_reached`, `/waypoint_status` | 10Hz / 1Hz | Detects arrival at predefined waypoints |
| `dashboard` | — | 1Hz | Live terminal dashboard (optional) |

### Launching the Monitoring Stack

```bash
# Full monitoring stack (health + waypoints)
ros2 launch watchdog_monitor watchdog.launch.py

# With terminal dashboard
ros2 launch watchdog_monitor watchdog.launch.py dashboard:=true

# Standalone dashboard only
ros2 launch watchdog_monitor dashboard.launch.py
```

---

## Waypoint Configuration

Edit `config/watchdog_params.yaml` (or override via launch):

```yaml
waypoints:
  ros__parameters:
    spawn_x: -21.0
    spawn_y: -47.0
    waypoints: [-19.0, -47.0, -15.0, -47.0, -21.0, -43.0]
    waypoint_names: ["WP-1", "WP-2", "WP-3"]
    arrival_radius: 0.5
```

> **Note:** `spawn_x` and `spawn_y` must match the robot's spawn position in the world. Waypoints are specified in world coordinates — the node automatically offsets odometry by the spawn position.

---

## Turret Vision (Face Detection Task)

> **Prerequisite:** The monitoring stack must be running before launching turret vision. It provides the waypoint events that trigger the task.

**Terminal 1 — start monitoring stack:**

```bash
ros2 launch watchdog_monitor watchdog.launch.py
```

**Terminal 2 — launch turret vision:**

```bash
ros2 launch turret_vision turret_vision.launch.py target_image:=/path/to/face.jpg
```

*Replace the `target_image` path with the absolute path to your target face image.*

### Turret Vision Nodes

| Node | Subscribes | Publishes | Description |
|------|------------|-----------|-------------|
| `recognition` | `/camera/image_raw`, `/capture_request` | `/match_found`, `/horizontal_error`, `/vertical_error` | InsightFace inference |
| `scanner` | `/start`, `/match_found`, `/horizontal_error`, `/vertical_error` | `/capture_request`, `/pan_deg`, `/tilt_deg`, `/laser_fire`, `/complete` | 21-position scan state machine |
| `turret` | `/pan_deg`, `/tilt_deg`, `/laser_fire` | `/turret_controller/commands` (sim) or serial (real) | Unified turret interface (auto-detects sim vs real) |
| `trigger` | `/waypoint_reached`, `/complete` | `/start`, `/done`, `/cmd_vel` | WP-2 listener, starts task, stops robot |

---

## Topics

| Topic | Type | Publisher |
|-------|------|-----------|
| `/system_status` | `std_msgs/String` (JSON) | `health` |
| `/system_alerts` | `std_msgs/String` (JSON) | `health` |
| `/waypoint_reached` | `std_msgs/String` (JSON) | `waypoints` |
| `/waypoint_status` | `std_msgs/String` (JSON) | `waypoints` |
| `/final_goal` | `geometry_msgs/PoseStamped` | External / operator |
| `/start` | `std_msgs/Bool` | `trigger` |
| `/complete` | `std_msgs/Bool` | `scanner` |
| `/done` | `std_msgs/Bool` | `trigger` |
| `/match_found` | `std_msgs/Bool` | `recognition` |
| `/capture_request` | `std_msgs/Bool` | `scanner` |
| `/pan_deg` | `std_msgs/Float32` | `scanner` |
| `/tilt_deg` | `std_msgs/Float32` | `scanner` |
| `/laser_fire` | `std_msgs/Bool` | `scanner` |
| `/horizontal_error` | `std_msgs/Float32` | `recognition` |
| `/vertical_error` | `std_msgs/Float32` | `recognition` |

---

## Sending Navigation Goal

Publish a goal directly to the navigation stack:

```bash
ros2 topic pub --once /final_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: <X>, y: <Y>}, orientation: {w: 1.0}}}"
```

Replace `<X>` and `<Y>` with target world coordinates (in metres, `map` frame).

**Example — drive to (24.5, −22.4):**

```bash
ros2 topic pub --once /final_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 24.5, y: -22.4}, orientation: {w: 1.0}}}"
```

> **`orientation: {w: 1.0}`** sets a neutral (zero-yaw) heading.  
> The planner determines the actual approach angle automatically.  
> To command a specific heading, use the quaternion formula:  
> `w = cos(θ/2)`, `z = sin(θ/2)` where θ is yaw in radians  
> (e.g., 90° → `z: 0.707, w: 0.707`).

---

## Manual Turret Control

To manually move the turret, publish angles directly:

```bash
# Pan left 30 degrees
ros2 topic pub --once /pan_deg std_msgs/msg/Float32 "{data: 30.0}"

# Tilt up 15 degrees
ros2 topic pub --once /tilt_deg std_msgs/msg/Float32 "{data: 15.0}"

# Fire laser
ros2 topic pub --once /laser_fire std_msgs/msg/Bool "{data: true}"
```

---

## Manually Trigger a Waypoint Event

```bash
ros2 topic pub --once /waypoint_reached std_msgs/msg/String \
  '{"data": "{\"event\": \"waypoint_reached\", \"waypoint\": {\"name\": \"WP-2\", \"index\": 2}}"}'
```

---

## Manually Start Face Detection Task

```bash
ros2 topic pub --once /start std_msgs/msg/Bool "{data: true}"
```

---

## Clean Build

```bash
rm -rf build/ install/ log/
colcon build
```

---

## Package Structure

```
src/
├── bringup/           # Launch files for sim/real
├── control/           # C++ control interfaces
├── description/       # URDF robot description
├── hardware/          # Real hardware drivers (LiDAR, IMU)
├── localization/      # EKF + SLAM
├── logger/            # Data logging
├── perception/        # Lane detection, costmaps, potholes
├── planning/          # Nav2 configuration
├── simulation/        # Gazebo world and models
├── turret_vision/     # Face detection + turret control
└── watchdog_monitor/  # System monitoring + waypoints