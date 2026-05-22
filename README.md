# MOLA SLAM Complete Guide
**Comprehensive Configuration and Usage Guide for ROS2 Humble on Ubuntu 22.04**

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![MOLA](https://img.shields.io/badge/MOLA-v2.2.0-00C853?style=for-the-badge)
![MRPT](https://img.shields.io/badge/MRPT-2.14.16-FF6F00?style=for-the-badge)
![mrpt_msgs](https://img.shields.io/badge/mrpt__msgs-0.5.0-FF6F00?style=for-the-badge)
![License](https://img.shields.io/badge/License-BSD--3-blue?style=for-the-badge)

**Primary Hardware:** Livox MID-360 LiDAR
**Focus:** GICP/ICP SLAM for Mapping and Localization
**Official Documentation:** https://docs.mola-slam.org/latest/
</div>

---

## Video Showcase

<table align="center">
  <tr>
    <td align="center">
      <a href="https://youtu.be/Iv2-noh69lY">
        <img src="https://img.youtube.com/vi/Iv2-noh69lY/maxresdefault.jpg" alt="Localization Demo" width="300">
      </a>
      <br>
      <strong>Localization via MOLA-LO</strong>
    </td>
    <td align="center">
      <a href="https://youtu.be/441KLE3GS2M">
        <img src="https://img.youtube.com/vi/441KLE3GS2M/maxresdefault.jpg" alt="Mapping Demo" width="300">
      </a>
      <br>
      <strong>Mapping with Livox MID-360</strong>
    </td>
  </tr>
</table>

---

## Table of Contents

- [Video Showcase](#video-showcase)
- [Installation](#installation)
  - [Quick Installation](#quick-installation)
- [Quick Start](#quick-start)
- [mola_bringup Launch Files](#mola_bringup-launch-files)
- [System Overview](#system-overview)
- [Configuration Files](#configuration-files)
- [Launch Commands](#launch-commands)
- [Pipeline Selection](#pipeline-selection)
- [Mapping Configuration](#mapping-configuration)
- [Localization Configuration](#localization-configuration)
- [State Estimator](#state-estimator)
- [ROS2 Topics and Transforms](#ros2-topics-and-transforms)
- [Map Operations](#map-operations)
- [Parameter Tuning](#parameter-tuning)
- [Performance Optimization](#performance-optimization)
- [Troubleshooting](#troubleshooting)
- [Common Issues](#common-issues)
- [Command Reference](#command-reference)
- [Examples](#examples)
- [Advanced Usage: Custom mola_bringup Package](#advanced-usage-custom-mola_bringup-package)
- [External Resources](#external-resources)

---

## Installation

### Quick Installation

```bash
# 1. Source ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. Clone the repository
cd ~
git clone https://github.com/Whan000/MOLA-SLAM.git
cd MOLA-SLAM/mola_ws

# 3. Install all dependencies
sudo apt update

sudo apt remove -y ros-humble-mrpt-*

sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions \
  libwxgtk3.0-gtk3-dev libwxgtk-media3.0-gtk3-dev \
  libopencv-dev \
  libeigen3-dev \
  libsuitesparse-dev \
  libassimp-dev \
  liboctomap-dev \
  libtbb-dev \
  libpcap-dev \
  libjpeg-dev \
  libglfw3-dev \
  libxrandr-dev \
  libxxf86vm-dev \
  libfreenect-dev \
  libopenni2-dev \
  libudev-dev \
  pybind11-dev \
  python3-pip \
  libcli11-dev \
  libtinyxml2-dev \
  ffmpeg libavcodec-dev libavformat-dev libswscale-dev \
  libyaml-cpp-dev \
  libboost-serialization-dev \
  libboost-system-dev \
  libboost-filesystem-dev \
  libboost-thread-dev \
  libboost-program-options-dev \
  libboost-date-time-dev \
  libboost-timer-dev \
  libboost-chrono-dev \
  libboost-regex-dev \
  zlib1g-dev \
  python3-matplotlib \
  python3-numpy \
  python3-tk \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-gtest \
  ros-humble-ament-cmake-python \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-sensor-msgs-py \
  ros-humble-nav-msgs \
  ros-humble-std-msgs \
  ros-humble-action-msgs \
  ros-humble-stereo-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-msgs \
  ros-humble-cv-bridge \
  ros-humble-rosbag2-cpp \
  ros-humble-rosbag2-storage \
  freeglut3-dev \
  ros-humble-rviz2

# 4. Build the workspace
colcon build --parallel-worker 4 --symlink-install --cmake-args -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_BUILD_TYPE=Release

# 5. Source the workspace
source install/setup.bash

# 6. (Optional) Add to .bashrc for permanent sourcing
echo "source ~/MOLA-SLAM/mola_ws/install/setup.bash" >> ~/.bashrc
```

---

### Prerequisites

```bash
# Ubuntu 22.04
lsb_release -a

# ROS2 Humble
source /opt/ros/humble/setup.bash
ros2 --version
```

### Important: Source ROS2 Environment

Before installing dependencies or building, ensure ROS2 Humble is sourced:

```bash
source /opt/ros/humble/setup.bash
```

**Add to .bashrc for automatic sourcing:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install All Dependencies

The workspace includes pre-configured MRPT packages (v2.14.16) and messages (v0.5.0) to build from source.

**⚠️ IMPORTANT:** Do NOT install `ros-humble-mrpt-*` packages from apt! The workspace includes MRPT source code and will build it from source. Installing both will cause conflicts.

Install all required system dependencies:

```bash
# Update package list
sudo apt update

sudo apt remove -y ros-humble-mrpt-*

# Install ALL dependencies in one command
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions \
  libwxgtk3.0-gtk3-dev libwxgtk-media3.0-gtk3-dev \
  libopencv-dev \
  libeigen3-dev \
  libsuitesparse-dev \
  libassimp-dev \
  liboctomap-dev \
  libtbb-dev \
  libpcap-dev \
  libjpeg-dev \
  libglfw3-dev \
  libxrandr-dev \
  libxxf86vm-dev \
  libfreenect-dev \
  libopenni2-dev \
  libudev-dev \
  pybind11-dev \
  python3-pip \
  libcli11-dev \
  libtinyxml2-dev \
  ffmpeg libavcodec-dev libavformat-dev libswscale-dev \
  libyaml-cpp-dev \
  libboost-serialization-dev \
  libboost-system-dev \
  libboost-filesystem-dev \
  libboost-thread-dev \
  libboost-program-options-dev \
  libboost-date-time-dev \
  libboost-timer-dev \
  libboost-chrono-dev \
  libboost-regex-dev \
  zlib1g-dev \
  python3-matplotlib \
  python3-numpy \
  python3-tk \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-gtest \
  ros-humble-ament-cmake-python \
  ros-humble-rclcpp \
  ros-humble-rclpy \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-sensor-msgs-py \
  ros-humble-nav-msgs \
  ros-humble-std-msgs \
  ros-humble-action-msgs \
  ros-humble-stereo-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-msgs \
  ros-humble-cv-bridge \
  ros-humble-rosbag2-cpp \
  ros-humble-rosbag2-storage \
  ros-humble-rviz2
```

**Note:** This installs 75+ packages including:
- **Build tools**: cmake, colcon, ament
- **MRPT dependencies**: wxWidgets, OpenGL, OpenCV, sensor drivers, CLI11 parser
- **MOLA dependencies**: YAML, Boost libraries (GTSAM is built from source in workspace)
- **ROS2 packages**: rclcpp/rclpy, message types, TF2, cv_bridge
- **Python tools**: matplotlib, numpy, tkinter

### Setup Workspace

This package includes all required source files. Simply extract/clone to your workspace:

```bash
# Create workspace
mkdir -p ~/mola_ws/src
cd ~/mola_ws/src

# Copy or clone the complete package here
# The package already includes:
#   - MOLA-SLAM/         (all MOLA packages + mola_bringup inside)
#   - mrpt_msgs-0.5.0/   (MRPT ROS2 messages)
#   - mrpt_ros-2.14.16/  (MRPT libraries for ROS2)
```

**Note:** `mola_bringup` is located **inside** MOLA-SLAM at:  
`~/mola_ws/src/MOLA-SLAM/src/mola_bringup/`

### Workspace Structure

Your workspace should have this structure:

```
~/mola_ws/
├── src/
│   ├── MOLA-SLAM/
│   │   └── src/
│   │       ├── mola/
│   │       ├── mola_bringup/              # Custom package (INSIDE MOLA-SLAM)
│   │       │   ├── mola_bringup/
│   │       │   │   ├── __init__.py
│   │       │   │   ├── filterpass.py
│   │       │   │   └── plot_lidar_trajectory.py
│   │       │   ├── launch/
│   │       │   │   ├── mola_slam_launch.py
│   │       │   │   └── mola_localize_launch.py
│   │       │   ├── package.xml
│   │       │   └── CMakeLists.txt
│   │       ├── mola_common/
│   │       ├── mola_imu_preintegration/
│   │       ├── mola_lidar_odometry/
│   │       ├── mola_state_estimation/
│   │       ├── mp2p_icp/
│   │       └── mrpt_ros_bridge/
│   ├── mrpt_msgs-0.5.0/                   # Pre-configured MRPT messages
│   └── mrpt_ros-2.14.16/                  # Pre-configured MRPT libraries
├── build/
├── install/
└── log/
```

**Important:** `mola_bringup` is located at `~/mola_ws/src/MOLA-SLAM/src/mola_bringup/`

### Build Workspace

Build in the correct order for dependency resolution:

```bash
cd ~/mola_ws

# Step 1: Build MRPT messages first
colcon build --symlink-install --cmake-args -DGTSAM_USE_SYSTEM_EIGEN=ON -DCMAKE_BUILD_TYPE=Release 

# Source the workspace
source install/setup.bash
```

### Verify Installation

```bash
# Check MOLA packages
ros2 pkg list | grep mola

# Check MRPT messages
ros2 interface list | grep mrpt_msgs

# Check mola_bringup
ros2 pkg list | grep mola_bringup

# Test map viewer
mm-viewer

# Test CLI interface
mola-lidar-odometry-cli --help

# Check launch files
ros2 launch mola_bringup --show-args
```

### Setup Auto-Sourcing

```bash
# Add to .bashrc for automatic sourcing
echo "source ~/mola_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Quick Start

### Step 1: Create a Map

```bash
cd ~/mola_ws
source install/setup.bash

# Launch SLAM workflow (includes filtering + visualization)
ros2 launch mola_bringup mola_slam_launch.py
```

**What happens:**
1. FilterPass removes noise from `/livox/lidar` → outputs to `/livox/lidar_filtered`
2. MOLA builds map from filtered point cloud
3. Trajectory GUI shows live position and ICP quality
4. RViz displays 3D map in real-time
5. MolaViz provides detailed SLAM visualization

**Drive your robot around the environment to build the map.**

**Save map (in another terminal):**
```bash
# Method 1: Save anytime with service call (recommended)
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/my_building'"

# This creates:
# - my_building.simplemap
# - my_building.mm
# - my_building.tum

# Method 2: Automatic save on shutdown (Ctrl+C)
# Creates files in current directory:
# - final_map.simplemap
# - estimated_trajectory.tum
```

### Step 2: Prepare Map for Localization

**If you used service call (Method 1):**
```bash
# Maps are already saved! Just verify:
ls -lh ~/mola_ws/map/my_building.*

# You should see:
# my_building.simplemap
# my_building.mm          ← Ready for localization!
# my_building.tum
```

**If you used automatic save (Method 2):**
```bash
# Convert simplemap to metric map (faster for localization)
sm2mm -i final_map.simplemap -o output_map.mm \
  --pipeline $(ros2 pkg prefix mp2p_icp)/share/mp2p_icp/demos/sm2mm_pointcloud_voxelize.yaml

# Create map directory and move maps there
mkdir -p ~/mola_ws/map
mv final_map.simplemap ~/mola_ws/map/my_building.simplemap
mv output_map.mm ~/mola_ws/map/my_building.mm
```

### Step 3: Update Map Paths

Before localization, edit the launch file to point to your maps:

```bash
# Edit the localization launch file
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# Update these lines (around line 11-12):
mm_map = '/home/YOUR_USERNAME/mola_ws/map/output_map.mm'
simple_map = '/home/YOUR_USERNAME/mola_ws/map/final_map.simplemap'

# Save and exit (Ctrl+X, Y, Enter)
```

**IMPORTANT:** Use absolute paths, not `~/` syntax!

### Step 4: Localize in the Map

```bash
# Launch localization workflow
ros2 launch mola_bringup mola_localize_launch.py
```

**Localization workflow:**
1. FilterPass processes point cloud
2. MOLA loads maps but stays **INACTIVE** (waiting for initial pose)
3. Trajectory GUI opens with initial pose controls
4. **You must provide initial pose:**
   - Enter X, Y, Z position (meters) where robot is currently located
   - Enter Yaw orientation (degrees)
   - Click "Publish Initial Pose & Activate MOLA"
5. MOLA **ACTIVATES** and starts localizing from your specified pose
6. Watch trajectory update in real-time
7. Toggle planar mode if needed (for slopes/3D motion)

---

## mola_bringup Launch Files

### Overview

The `mola_bringup` package provides **two launch files** for complete SLAM workflows:

| Launch File | Purpose | MOLA Mode | Output |
|-------------|---------|-----------|--------|
| **mola_slam_launch.py** | Create maps | Mapping (active) | Saves `.simplemap` and `.tum` files |
| **mola_localize_launch.py** | Localize in existing map | Localization (inactive until pose given) | No map output |

### Launch File 1: mola_slam_launch.py

**Purpose:** Build a new map of your environment

**What it launches:**
1. `filterpass.py` - Point cloud preprocessing
2. MOLA SLAM (via `ros2-lidar-odometry-katana.launch.py`)
   - Topic: `/livox/lidar_filtered`
   - IMU: `/imu`
   - RViz: Enabled
   - MolaViz GUI: Enabled
   - **Mapping: ENABLED** (`start_mapping_enabled:=true`)
   - **Active: YES** (`start_active:=true`) - starts immediately
3. `plot_lidar_trajectory.py` - Real-time visualization

**Usage:**
```bash
ros2 launch mola_bringup mola_slam_launch.py
```

**Output:**
- `final_map.simplemap` (in current directory)
- `estimated_trajectory.tum` (in current directory)

**Configuration:**
- Filterpass uses default parameters (360° FOV, intensity 1-255)
- MOLA starts mapping immediately
- No initial pose needed

---

### Launch File 2: mola_localize_launch.py

**Purpose:** Localize robot in a previously built map

**What it launches:**
1. `filterpass.py` - Point cloud preprocessing  
2. MOLA Localization (via `ros2-lidar-odometry-localize-katana.launch.py`)
   - Topic: `/livox/lidar_filtered`
   - IMU: `/imu`
   - RViz: Enabled
   - MolaViz GUI: Enabled
   - **Mapping: DISABLED** (`enable_mapping:=false`)
   - **Active: NO** (`start_active:=false`) - waits for initial pose
   - Loads maps from these paths:
     ```python
     mm_map = '/home/katana/Desktop/array/mola_ws/map/FIBO.mm'
     simple_map = '/home/katana/Desktop/array/mola_ws/map/FIBO.simplemap'
     ```
3. `plot_lidar_trajectory.py` - Visualization + initial pose GUI

**CRITICAL:** You **MUST** update the map paths before using this launch file!

**Usage:**
```bash
# 1. Edit launch file with your map paths
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# 2. Update lines 11-12 to your maps:
mm_map = '/home/YOUR_USERNAME/mola_ws/map/YOUR_MAP.mm'
simple_map = '/home/YOUR_USERNAME/mola_ws/map/YOUR_MAP.simplemap'

# 3. Launch
ros2 launch mola_bringup mola_localize_launch.py

# 4. In the GUI that opens:
#    - Enter initial pose (X, Y, Z, Yaw)
#    - Click "Publish Initial Pose & Activate MOLA"
#    - MOLA will activate and start localizing
```

**Important Notes:**
- MOLA stays **inactive** until you publish initial pose
- You must know approximate starting position
- GUI provides controls for activation
- No map files are generated

---

### Comparison Table

| Feature | mola_slam_launch.py | mola_localize_launch.py |
|---------|---------------------|-------------------------|
| **Use Case** | Map building | Localization |
| **MOLA Mode** | Mapping enabled | Mapping disabled |
| **Start State** | Active immediately | Inactive (waits for pose) |
| **Initial Pose** | Not needed | **Required via GUI** |
| **Maps Required** | None | **Yes - must configure paths** |
| **Output Files** | `.simplemap`, `.tum` | None |
| **GUI Purpose** | Trajectory visualization | Pose setting + visualization |

---

## System Overview

### Architecture

MOLA (Modular Optimization framework for Localization and mApping) provides LiDAR Odometry (LO), LiDAR-inertial Odometry (LIO), SLAM, localization-only modes, and geo-referencing.

### Key Features

- **Multiple ICP Algorithms:** GICP (high accuracy), ICP (fast), NDT
- **State Estimation:** Simple pass-through or factor graph smoother (GTSAM backend)
- **Map Formats:** Simplemap (keyframe-based) and metric map (dense point cloud)
- **ROS2 Native:** Full integration with ROS2 Humble
- **Real-time Performance:** 5-20 Hz depending on configuration

### Pipeline Comparison

| Feature | GICP | ICP |
|---------|------|-----|
| **Algorithm** | Covariance-to-covariance | Point-to-point |
| **Speed** | 5-10 Hz | 10-20 Hz |
| **Accuracy** | Higher | Lower |
| **Best For** | Outdoor, unstructured | Indoor, structured |
| **CPU Load** | Higher | Lower |
| **Memory** | Higher | Lower |

**Recommendation:** Use GICP for Livox MID-360 outdoor mapping

---

## Configuration Files

### File Structure

```
~/mola_ws/src/MOLA-SLAM/src/mola_lidar_odometry/
│
├── pipelines/                              # Algorithm configurations
│   ├── lidar3d-gicp-katana.yaml           # GICP pipeline (recommended)
│   ├── lidar3d-katana.yaml                # ICP pipeline
│   ├── lidar3d-icp.yaml                   # Basic ICP
│   └── lidar3d-ndt.yaml                   # NDT algorithm
│
├── mola-cli-launchs/
│   └── lidar_odometry_ros2.yaml           # ROS2 system configuration
│
├── state-estimator-params/
│   ├── state-estimation-simple.yaml       # Direct pass-through
│   └── state-estimation-smoother.yaml     # Factor graph optimization
│
└── ros2-launchs/
    ├── ros2-lidar-odometry-katana.launch.py         # Mapping mode
    └── ros2-lidar-odometry-localize-katana.launch.py # Localization mode
```

### Configuration Layers

**Layer 1: Launch File** - ROS2 arguments and environment variables  
**Layer 2: System Config** - Module definitions and ROS2 topics  
**Layer 3: Pipeline Config** - ICP algorithm and point cloud processing  
**Layer 4: State Estimator Config** - Factor graph parameters

---

## Launch Commands

### Recommended: Using mola_bringup (Integrated Workflow)

**1. Mapping - Build a New Map:**

```bash
ros2 launch mola_bringup mola_slam_launch.py
```

**Includes:**
- Point cloud filtering (removes noise)
- MOLA SLAM (mapping mode, starts active)
- Real-time trajectory visualization
- RViz and MolaViz
- Auto-saves map when you stop

**2. Localization - Use Existing Map:**

```bash
# FIRST: Update map paths in launch file!
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# Then launch:
ros2 launch mola_bringup mola_localize_launch.py

# In GUI: Enter initial pose and click "Publish Initial Pose & Activate MOLA"
```

**Includes:**
- Point cloud filtering
- MOLA localization (starts inactive, you activate via GUI)
- Initial pose GUI control
- Planar motion toggle
- Real-time visualization

---

### Alternative: Direct MOLA Launch (Without mola_bringup)

Use these if you don't need point cloud filtering or custom visualization:

**Basic Mapping:**

```bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
  lidar_topic_name:=/livox/lidar
```

**Mapping with All Sensors:**

```bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
  lidar_topic_name:=/livox/lidar \
  imu_topic_name:=/livox/imu \
  gnss_topic_name:=/gps \
  generate_simplemap:=True \
  use_state_estimator:=True
```

**Localization:**

```bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry-localize-katana.launch.py \
  lidar_topic_name:=/livox/lidar \
  start_mapping_enabled:=False \
  mola_initial_map_mm_file:=/path/to/map.mm
```

---

### Common Launch Arguments (for direct MOLA launches)

| Argument | Default | Description |
|----------|---------|-------------|
| `lidar_topic_name` | **Required** | LiDAR PointCloud2 topic |
| `imu_topic_name` | `/livox/imu` | IMU topic |
| `gnss_topic_name` | `/gps` | GNSS topic |
| `start_mapping_enabled` | `True` | Enable/disable mapping |
| `use_state_estimator` | `True` | Use smoother estimator |
| `use_rviz` | `True` | Launch RViz2 |
| `use_mola_gui` | `True` | Launch MolaViz GUI |
| `generate_simplemap` | `False` | Save map file |
| `enforce_planar_motion` | `True` | Lock z, pitch, roll |
| `mola_lo_pipeline` | `../pipelines/lidar3d-katana.yaml` | Pipeline config file |

---

## Pipeline Selection

### Available Pipelines

Edit pipeline via launch argument:

```bash
mola_lo_pipeline:=../pipelines/lidar3d-gicp-katana.yaml  # High accuracy
mola_lo_pipeline:=../pipelines/lidar3d-katana.yaml       # Fast
```

Or edit system config: `mola-cli-launchs/lidar_odometry_ros2.yaml`

```yaml
modules:
  - name: lidar_odom
    type: mola::LidarOdometry
    params: "${MOLA_ODOMETRY_PIPELINE_YAML|../pipelines/lidar3d-gicp-katana.yaml}"
```

### GICP vs ICP Parameters

| Parameter | GICP | ICP |
|-----------|------|-----|
| `min_icp_goodness` | 0.60 | 0.25 |
| `initial_sigma` | 0.5 m | 2.0 m |
| `maximum_sigma` | 2.0 m | 3.0 m |
| `alpha` | 0.99 | 0.90 |
| `optimize_twist` | false | true |

---

## Mapping Configuration

### Essential Parameters

Edit: `pipelines/lidar3d-gicp-katana.yaml`

```yaml
# Map generation
local_map_updates:
  enabled: true
  min_translation_between_keyframes: 0.5    # [m]
  min_rotation_between_keyframes: 5.0       # [deg]
  max_distance_to_keep_keyframes: 200       # [m] (0 = unlimited)
  check_for_removal_every_n: 100

# Simplemap output
simplemap:
  generate: true
  save_final_map_to_file: 'final_map.simplemap'
  min_translation_between_keyframes: 0.5    # [m]
  min_rotation_between_keyframes: 5.0       # [deg]

# ICP quality threshold
min_icp_goodness: 0.60  # GICP: 0.60, ICP: 0.25

# Adaptive threshold (KISS-ICP inspired)
adaptive_threshold:
  enabled: true
  initial_sigma: 0.5      # [m]
  min_motion: 0.5         # [m]
  maximum_sigma: 2.0      # [m]
  alpha: 0.99

# Point cloud decimation
observations_filter_1st_pass:
  - class_name: mp2p_icp_filters::FilterDecimateAdaptive
    params:
      desired_output_point_count: 5000
      voxel_size: 0.2     # [m]
```

### Keyframe Density

**Dense map (better localization, higher memory):**
```yaml
min_translation_between_keyframes: 0.3  # From 0.5
min_rotation_between_keyframes: 3.0     # From 5.0
```

**Sparse map (lower memory, faster):**
```yaml
min_translation_between_keyframes: 1.0  # From 0.5
min_rotation_between_keyframes: 10.0    # From 5.0
```

### ICP Quality

**Stricter (fewer bad matches):**
```yaml
min_icp_goodness: 0.70  # From 0.60
```

**Relaxed (accept more matches):**
```yaml
min_icp_goodness: 0.40  # From 0.60
```

### Point Cloud Processing

**High accuracy (slower):**
```yaml
desired_output_point_count: 8000  # From 5000
voxel_size: 0.15                  # From 0.2
```

**Fast processing (lower accuracy):**
```yaml
desired_output_point_count: 3000  # From 5000
voxel_size: 0.3                   # From 0.2
```

---

## Localization Configuration

### Load Existing Map

```bash
# Method 1: Load metric map (recommended - faster)
mola_initial_map_mm_file:=/absolute/path/to/map.mm

# Method 2: Load simplemap
mola_initial_map_sm_file:=/absolute/path/to/final_map.simplemap
```

### Initial Pose Configuration

Edit: `pipelines/lidar3d-gicp-katana.yaml`

```yaml
initial_localization:
  method: "InitLocalization::FixedPose"  # or "InitLocalization::FromImu"
  fixed_initial_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x y z yaw pitch roll
```

**Set via launch:**
```bash
initial_localization_method:=InitLocalization::FixedPose
```

### Wheel Odometry Integration

```bash
forward_ros_tf_odom_to_mola:=True
```

This imports `/tf` transform `odom -> base_link` as motion prior for MOLA.

---

## State Estimator

### Estimator Selection

**Simple (direct pass-through):**
```bash
use_state_estimator:=False
```
- No filtering or smoothing
- Lowest latency
- Direct LiDAR odometry output

**Smoother (recommended):**
```bash
use_state_estimator:=True
```
- Factor graph optimization (GTSAM backend)
- Integrates LiDAR + IMU + odometry
- Smoother trajectory
- Slightly higher latency

### Smoother Configuration

Edit: `state-estimator-params/state-estimation-smoother.yaml`

```yaml
params:
  # Time window
  sliding_window_length: 2.5  # [s]
  
  # Process noise
  sigma_random_walk_acceleration_linear: 1.0    # [m/s²]
  sigma_random_walk_acceleration_angular: 10.0  # [rad/s²]
  
  # Integrator noise
  sigma_integrator_position: 0.1     # [m]
  sigma_integrator_orientation: 1.0  # [rad]
  
  # Velocity model timeout
  max_time_to_use_velocity_model: 0.75  # [s]
  
  # Planar motion constraint (for ground robots)
  enforce_planar_motion: true
  
  # Initial velocity
  initial_twist: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # vx vy vz wx wy wz
```

### Tuning for Different Scenarios

**High-speed motion:**
```yaml
max_time_to_use_velocity_model: 1.0
sigma_random_walk_acceleration_linear: 2.0
```

**Smoother trajectory (more filtering):**
```yaml
sigma_random_walk_acceleration_linear: 0.5
sigma_random_walk_acceleration_angular: 5.0
sliding_window_length: 3.0
```

**3D motion (disable planar constraint):**
```yaml
enforce_planar_motion: false
```

**Runtime Toggle via GUI:**
The planar motion constraint can be toggled at runtime using the trajectory visualization GUI (`plot_lidar_trajectory.py`). Click "Enable Planar Mode" or "Disable Planar Mode" buttons to change the constraint without restarting MOLA. This is useful when transitioning between flat terrain and slopes during operation.

---

## ROS2 Topics and Transforms

### System Configuration

Edit: `mola-cli-launchs/lidar_odometry_ros2.yaml`

```yaml
params:
  # Frame names
  base_link_frame: "base_link"
  base_footprint_frame: "base_footprint"
  odom_frame: "odom"
  reference_frame: "map"
  
  # REP105 compliance (map -> odom -> base_link)
  publish_localization_following_rep105: true
  
  # Publishing
  publish_odometry_msgs_from_slam: true
  publish_tf_from_slam: true
  
  # Sensor subscriptions
  subscribe:
    - topic: /livox/lidar
      msg_type: PointCloud2
      output_sensor_label: "lidar"
      use_fixed_sensor_pose: false  # Read from /tf
      
    - topic: /livox/imu
      msg_type: Imu
      output_sensor_label: "imu"
```

### Transform Tree

```
map                    # Global reference
 └─ odom              # Drift-free locally
     └─ base_footprint # Ground projection
         └─ base_link  # Robot center
             └─ lidar_link
```

### Configure base_footprint → base_link

```bash
mola_footprint_to_base_link_tf:="[0.0, 0.0, 0.3, 0, 0, 0]"
# Format: [x, y, z, yaw_deg, pitch_deg, roll_deg]
```

### Fixed Sensor Poses

If not using `/tf` for sensor transforms:

```yaml
subscribe:
  - topic: /livox/lidar
    fixed_sensor_pose: "0.2 0 0.5 0 0 0"  # x y z yaw pitch roll
    use_fixed_sensor_pose: true
```

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mola_lidar_odometry/odom` | `nav_msgs/Odometry` | Pose estimate |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/mola_lidar_odometry/local_map` | `sensor_msgs/PointCloud2` | Local map |
| `/mola_lidar_odometry/current_scan` | `sensor_msgs/PointCloud2` | Deskewed scan |

---

## Map Operations

### Save Map

**Method 1: Service Call (Recommended - Save Anytime)**

Save map during operation without stopping MOLA:

```bash
# Save map with custom name and path
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/my_map'"

# This creates three files:
# - my_map.simplemap  (keyframe-based map)
# - my_map.mm         (metric map - optional)
# - my_map.tum        (trajectory in TUM format)
```

**Advantages:**
- Save map while SLAM is still running
- Save multiple snapshots at different locations
- Specify exact output path and name
- No need to stop the robot

**Usage during mapping:**
```bash
# 1. Start mapping
ros2 launch mola_bringup mola_slam_launch.py

# 2. Drive around and build map

# 3. Save map whenever you want (in another terminal)
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/building_floor_1'"

# 4. Continue mapping or save another snapshot
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/building_floor_2'"

# 5. When done, Ctrl+C
```

**Method 2: Automatic Save on Shutdown**

Maps are automatically saved when you stop MOLA (Ctrl+C):

```bash
ros2 launch mola_bringup mola_slam_launch.py

# Drive robot around
# Ctrl+C when done
# Output files in current directory:
# - final_map.simplemap
# - estimated_trajectory.tum
```

**Note:** Automatic save uses fixed filenames. Use service call for custom names.

### Convert Map

```bash
# Convert simplemap to metric map for faster localization:
sm2mm -i final_map.simplemap -o output_map.mm \
  --pipeline $(ros2 pkg prefix mp2p_icp)/share/mp2p_icp/demos/sm2mm_pointcloud_voxelize.yaml

# Verify:
sm-cli info final_map.simplemap
ls -lh output_map.mm
```

**Note:** The `sm2mm` tool uses a YAML pipeline configuration file. Available pipelines in `mp2p_icp/demos/`:
- `sm2mm_pointcloud_voxelize.yaml` - Voxelized point cloud (recommended, balanced)
- `sm2mm_bonxai_voxelmap.yaml` - Bonxai voxel map (memory efficient)
- `sm2mm_bonxai_voxelmap_gridmap.yaml` - Voxel + 2D grid map
- See [sm2mm documentation](https://docs.mola-slam.org/latest/app_sm2mm.html) for details

### View Map

```bash
# View metric map in 3D:
mm-viewer output_map.mm

# View simplemap info:
sm-cli info final_map.simplemap
```

**Example output:**
```
Number of keyframes: 245
Total points: 1,234,567
Map bounds: x[-50.2, 150.3], y[-80.1, 20.5], z[-0.5, 2.3]
```

### Load Map

Map loading is configured in `mola_localize_launch.py`:

```python
# Edit this file before launching localization
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# Update these lines:
mm_map = '/home/YOUR_USERNAME/mola_ws/map/output_map.mm'
simple_map = '/home/YOUR_USERNAME/mola_ws/map/final_map.simplemap'
```

---

## Parameter Tuning

### ICP Quality Threshold

Controls when ICP matches are accepted.

**Location:** `pipelines/lidar3d-gicp-katana.yaml`

```yaml
min_icp_goodness: 0.60  # GICP: 0.60, ICP: 0.25
```

**Effects:**
- **Higher (0.70-0.80):** Only high-quality matches, may skip valid data
- **Lower (0.40-0.50):** Accept more matches, may include drift

### Adaptive Threshold

Automatically adjusts ICP search radius (KISS-ICP methodology).

```yaml
adaptive_threshold:
  enabled: true
  initial_sigma: 0.5      # [m] Starting search radius
  minimum_sigma: 0.1      # [m] Never go below this
  maximum_sigma: 2.0      # [m] Never exceed this
  alpha: 0.99             # Exponential smoothing (0.5=fast, 0.99=slow)
```

**Tuning:**
- `alpha` near 1.0: Smooth, slow adaptation
- `alpha` near 0.5: Reactive, fast adaptation

### Keyframe Density

Controls map density and memory usage.

```yaml
local_map_updates:
  min_translation_between_keyframes: 0.5  # [m]
  min_rotation_between_keyframes: 5.0     # [deg]

simplemap:
  min_translation_between_keyframes: 0.5  # [m]
  min_rotation_between_keyframes: 5.0     # [deg]
```

**Guidelines:**
- **Dense (0.2-0.3 m):** Better localization, high memory
- **Balanced (0.5 m):** Recommended
- **Sparse (1.0+ m):** Low memory, may lose features

### Point Cloud Decimation

```yaml
observations_filter_1st_pass:
  - class_name: mp2p_icp_filters::FilterDecimateAdaptive
    params:
      desired_output_point_count: 5000  # Target points
      voxel_size: 0.2                   # [m] Grid size
```

**Voxel Size:**
- 0.1 m: Very dense (indoor small rooms)
- 0.2 m: Balanced (recommended)
- 0.3-0.5 m: Coarse (outdoor large-scale)

**Point Count:**
- 3000: Fast, low accuracy
- 5000: Balanced (recommended)
- 8000+: Slow, high accuracy

### Map Size Limit

Prevents unlimited memory growth.

```yaml
local_map_updates:
  max_distance_to_keep_keyframes: 200  # [m] (0 = unlimited)
  check_for_removal_every_n: 100
```

**Recommendations:**
- **Indoor:** 50-100 m
- **Outdoor:** 200-500 m
- **Unlimited:** 0 (use with caution)

---

## Performance Optimization

### Increase Speed (Lower Accuracy)

```yaml
# In pipeline YAML:
min_icp_goodness: 0.40                    # From 0.60
desired_output_point_count: 3000          # From 5000
voxel_size: 0.3                           # From 0.2

visualization:
  map_update_decimation: 10               # From 1
```

**Switch to ICP:**
```bash
mola_lo_pipeline:=../pipelines/lidar3d-katana.yaml
```

### Increase Accuracy (Slower)

```yaml
# In pipeline YAML:
min_icp_goodness: 0.70                    # From 0.60
desired_output_point_count: 8000          # From 5000
voxel_size: 0.15                          # From 0.2
min_translation_between_keyframes: 0.3    # From 0.5
```

### Reduce Memory Usage

```yaml
# Limit map size:
local_map_updates:
  max_distance_to_keep_keyframes: 100     # From 200

# Reduce points:
desired_output_point_count: 3000

# Sparse keyframes:
min_translation_between_keyframes: 1.0    # From 0.5
```

---

## Troubleshooting

### Low ICP Quality Warnings

**Symptoms:**
```
[WARN] ICP quality below threshold: 0.35 < 0.60
```

**Solutions:**

1. Lower threshold:
```yaml
min_icp_goodness: 0.40
```

2. Increase search radius:
```yaml
adaptive_threshold:
  maximum_sigma: 3.0
```

3. Increase point count:
```yaml
desired_output_point_count: 8000
```

### Map Drift in Corridors

**Solutions:**

1. Disable adaptive threshold:
```yaml
adaptive_threshold:
  enabled: false
```

2. Increase keyframe density:
```yaml
min_translation_between_keyframes: 0.3
```

3. Use GICP:
```bash
mola_lo_pipeline:=../pipelines/lidar3d-gicp-katana.yaml
```

### System Too Slow

**Solutions:**

1. Reduce point count:
```yaml
desired_output_point_count: 3000
voxel_size: 0.3
```

2. Switch to ICP:
```bash
mola_lo_pipeline:=../pipelines/lidar3d-katana.yaml
```

3. Limit map size:
```yaml
max_distance_to_keep_keyframes: 50
```

### Cannot Localize

**Check:**

1. Map file paths correct in launch file:
```python
# Check these paths exist:
ls -lh /home/YOUR_USERNAME/mola_ws/map/output_map.mm
ls -lh /home/YOUR_USERNAME/mola_ws/map/final_map.simplemap
```

2. Initial pose provided via GUI

3. MOLA activated via GUI button

### Transform Issues

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**Expected chain:**
```
map -> odom -> base_footprint -> base_link -> lidar_link
```

### No Sensor Data

**Check topics:**
```bash
ros2 topic list | grep lidar
ros2 topic hz /livox/lidar
ros2 topic hz /livox/lidar_filtered  # After filtering
ros2 topic echo /livox/lidar
```

**Verify topic name:**
```bash
lidar_topic_name:=/livox/lidar  # With leading slash
```

### Build Errors

**Clean rebuild:**
```bash
cd ~/mola_ws
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

**Rebuild specific packages:**
```bash
# Rebuild MRPT messages
colcon build --packages-select mrpt_msgs --symlink-install

# Rebuild MRPT libraries
colcon build --packages-select mrpt_lib* mrpt_apps --symlink-install

# Rebuild MOLA
colcon build --packages-select mola_* --symlink-install

# Rebuild mola_bringup
colcon build --packages-select mola_bringup --symlink-install
```

---

## Common Issues

| Problem | Quick Solution |
|---------|---------------|
| Save map during run | Use service: `ros2 service call /map_save mola_msgs/srv/MapSave "map_path: '/path/to/map'"` |
| Low ICP quality | Lower `min_icp_goodness` to 0.40 |
| Map drift | Disable `adaptive_threshold` |
| Slow performance | Reduce `desired_output_point_count` to 3000 |
| High memory | Set `max_distance_to_keep_keyframes: 100` |
| Can't localize | Check map paths in launch file, provide initial pose via GUI |
| MOLA won't activate | Click "Publish Initial Pose & Activate MOLA" in GUI |
| No transforms | Run `ros2 run tf2_tools view_frames` |
| No sensor data | Check `ros2 topic hz /livox/lidar` |
| No filtered data | Check `ros2 topic hz /livox/lidar_filtered` |
| Build errors | Clean rebuild: `rm -rf build/ install/ log/` |
| Workspace not sourced | `source ~/mola_ws/install/setup.bash` |
| MRPT not found | Rebuild: `colcon build --packages-select mrpt_*` |
| mrpt_msgs missing | Rebuild: `colcon build --packages-select mrpt_msgs` |
| mola_bringup missing | Rebuild: `colcon build --packages-select mola_bringup` |
| Map file not found | Use absolute paths, check file exists with `ls -lh` |

---

## Command Reference

### Workspace Management

```bash
# Source workspace (every terminal):
source ~/mola_ws/install/setup.bash

# Add to .bashrc:
echo "source ~/mola_ws/install/setup.bash" >> ~/.bashrc

# Navigate to workspace:
cd ~/mola_ws
```

### Launch

```bash
# Mapping (recommended):
ros2 launch mola_bringup mola_slam_launch.py

# Localization (recommended):
ros2 launch mola_bringup mola_localize_launch.py
```

### Map Tools

```bash
# Save map during operation (recommended):
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/my_map'"

# Convert:
sm2mm -i final_map.simplemap -o output_map.mm \
  --pipeline $(ros2 pkg prefix mp2p_icp)/share/mp2p_icp/demos/sm2mm_pointcloud_voxelize.yaml

# View:
mm-viewer output_map.mm

# Info:
sm-cli info final_map.simplemap
```

### Build

```bash
# Build all packages (correct order):
cd ~/mola_ws

# Step 1: MRPT messages
colcon build --packages-select mrpt_msgs --symlink-install

# Step 2: MRPT libraries
colcon build --packages-select mrpt_lib* mrpt_apps --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Step 3: MOLA packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Step 4: mola_bringup
colcon build --packages-select mola_bringup --symlink-install

# Or build everything at once:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Clean build:
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Debug

```bash
# Check topics:
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /livox/lidar_filtered
ros2 topic echo /livox/lidar

# Check TF:
ros2 run tf2_tools view_frames
evince frames.pdf

# Check nodes:
ros2 node list
ros2 node info /mola_launcher

# Check MRPT messages:
ros2 interface list | grep mrpt_msgs

# Check packages:
ros2 pkg list | grep mola
ros2 pkg list | grep mrpt

# Monitor system:
htop
watch -n 1 'ros2 topic hz /livox/lidar'
```

---

## Examples

### Example 1: Create Map with Service Save

```bash
cd ~/mola_ws
source install/setup.bash

# Launch integrated SLAM workflow
ros2 launch mola_bringup mola_slam_launch.py

# Drive robot around environment
# In another terminal, save map at any time:
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/my_building'"

# Output files:
# - my_building.simplemap
# - my_building.mm (if generated)
# - my_building.tum

# Continue mapping or Ctrl+C when done
```

### Example 2: Save and Organize Maps

```bash
# During mapping, save with service call:
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/my_building'"

# Or if using automatic save (after Ctrl+C):
# Convert simplemap to metric map
sm2mm -i final_map.simplemap -o output_map.mm \
  --pipeline $(ros2 pkg prefix mp2p_icp)/share/mp2p_icp/demos/sm2mm_pointcloud_voxelize.yaml

# Move to organized location
mkdir -p ~/mola_ws/map
mv final_map.simplemap ~/mola_ws/map/my_building.simplemap
mv output_map.mm ~/mola_ws/map/my_building.mm

# View the map
mm-viewer ~/mola_ws/map/my_building.mm

# Check map info
sm-cli info ~/mola_ws/map/my_building.simplemap
```

### Example 3: Localize with mola_bringup

```bash
# Step 1: Edit launch file with your map paths
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# Change lines 11-12 to:
# mm_map = '/home/YOUR_USERNAME/mola_ws/map/my_building.mm'
# simple_map = '/home/YOUR_USERNAME/mola_ws/map/my_building.simplemap'

# Step 2: Launch localization
ros2 launch mola_bringup mola_localize_launch.py

# Step 3: In GUI that opens:
# - Move robot to known location in map
# - Enter X, Y, Z position (meters)
# - Enter Yaw orientation (degrees)
# - Click "Publish Initial Pose & Activate MOLA"

# Step 4: Watch robot localize in real-time
# - Trajectory plot shows position
# - ICP quality plot shows localization confidence
# - Toggle planar mode if robot is on slopes
```

### Example 4: Custom Filtering Parameters

Edit `mola_slam_launch.py` to customize filterpass:

```python
filterpass = Node(
    package='mola_bringup',
    executable='filterpass.py',
    name='filterpass',
    parameters=[{
        'angle_deg': 10.0,          # Remove ±10° cone
        'keep_within': False,        # Remove points inside cone
        'min_intensity': 30.0,       # Strong returns only
        'max_intensity': 250.0
    }],
    output='screen'
)
```

Then rebuild and launch:

```bash
colcon build --packages-select mola_bringup --symlink-install
source install/setup.bash
ros2 launch mola_bringup mola_slam_launch.py
```

### Example 5: Direct MOLA Launch (No Preprocessing)

For testing without mola_bringup:

```bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry-katana.launch.py \
  lidar_topic_name:=/livox/lidar \
  imu_topic_name:=/imu \
  use_rviz:=true \
  use_mola_gui:=true \
  generate_simplemap:=True
```

### Example 6: Multiple Mapping Sessions

```bash
# Create different maps for different areas

# Session 1: Building A
ros2 launch mola_bringup mola_slam_launch.py
# Drive around
# Save map with service call:
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/building_a'"
# Ctrl+C

# Session 2: Building B  
ros2 launch mola_bringup mola_slam_launch.py
# Drive around
# Save map with service call:
ros2 service call /map_save mola_msgs/srv/MapSave \
  "map_path: '/home/katana/mola_ws/map/building_b'"
# Ctrl+C

# Now you have two maps:
ls ~/mola_ws/map/
# building_a.simplemap, building_a.mm, building_a.tum
# building_b.simplemap, building_b.mm, building_b.tum

# Update mola_localize_launch.py to use the appropriate map
```

---

## Advanced Usage: Custom mola_bringup Package

This section documents the custom `mola_bringup` package which provides preprocessing, visualization, and enhanced control capabilities for MOLA SLAM.

### Package Overview

The `mola_bringup` package includes:

1. **filterpass.py** - LiDAR point cloud preprocessing with angular FOV and intensity filtering
2. **plot_lidar_trajectory.py** - Real-time trajectory visualization with GUI control panel
3. **Two launch files:**
   - `mola_slam_launch.py` - For creating maps
   - `mola_localize_launch.py` - For localization in existing maps

### Package Structure

```
~/mola_ws/src/MOLA-SLAM/src/mola_bringup/
├── mola_bringup/
│   ├── __init__.py
│   ├── filterpass.py                    # Point cloud filter node
│   └── plot_lidar_trajectory.py         # Trajectory plotter with GUI
├── launch/
│   ├── mola_slam_launch.py              # SLAM workflow (mapping)
│   └── mola_localize_launch.py          # Localization workflow
├── package.xml
├── CMakeLists.txt
└── README.md
```

---

### Node 1: filterpass.py

**Purpose:** Real-time LiDAR point cloud preprocessing with dual filtering capabilities.

#### Features

- **Angular Field of View Filtering**
  - Filter points by azimuth angle (±θ degrees)
  - Keep or remove points within specified cone
  - Useful for removing robot body, specific directions, or focusing on specific areas

- **Intensity Range Filtering**
  - Filter points by intensity values
  - Remove noise, weak returns, or over-saturated points
  - Improve point cloud quality for ICP matching

#### Default Parameters (in launch files)

```yaml
input_topic: '/livox/lidar'           # Input PointCloud2 topic
output_topic: '/livox/lidar_filtered' # Output PointCloud2 topic (fed to MOLA)
angle_deg: 360.0                      # Full 360° FOV (no angular filtering)
keep_within: true                     # Keep all points
min_intensity: 1.0                    # Accept low intensity
max_intensity: 255.0                  # Accept all intensities
```

#### Customizing Filter Parameters

To customize, edit the launch file and add parameters:

```python
# In mola_slam_launch.py or mola_localize_launch.py:
filterpass = Node(
    package='mola_bringup',
    executable='filterpass.py',
    name='filterpass',
    parameters=[{
        'angle_deg': 10.0,      # ±10° cone
        'keep_within': False,   # Remove points inside cone
        'min_intensity': 20.0,  # Remove weak returns
        'max_intensity': 250.0
    }],
    output='screen'
)
```

#### Usage Examples

**Example 1: Remove Robot Body (Front 20°)**

```python
parameters=[{
    'angle_deg': 10.0,         # ±10° = 20° total cone
    'keep_within': False,      # Remove points inside
    'min_intensity': 10.0,
    'max_intensity': 200.0
}]
```

**Example 2: Forward-Looking Only (90° Cone)**

```python
parameters=[{
    'angle_deg': 45.0,         # ±45° = 90° total cone
    'keep_within': True,       # Keep only points inside
    'min_intensity': 5.0
}]
```

**Example 3: High-Intensity Points Only**

```python
parameters=[{
    'min_intensity': 50.0,     # Strong returns only
    'max_intensity': 255.0
}]
```

#### Technical Details

**Coordinate System:**
- Azimuth angle: `atan2(y, x)` in LiDAR frame
- 0° = forward (+X axis)
- ±180° = backward (-X axis)
- ±90° = sides (±Y axis)

**QoS Settings:**
- Uses system default QoS (auto-negotiating)
- Compatible with both Reliable and Best Effort publishers

**Performance:**
- Overhead: ~5-10 ms per scan (typical)
- Logs filter statistics every 5000 points
- Warns if zero points pass filter

---

### Node 2: plot_lidar_trajectory.py

**Purpose:** Real-time trajectory visualization with advanced MOLA control capabilities through an integrated GUI.

#### Features

**Visualization:**
- 2D Trajectory (XY plane)
- 3D Trajectory (XZ and YZ projections)
- ICP Quality monitoring over time
- Orientation tracking (Roll, Pitch, Yaw)
- 6 live plots updated at 10 Hz

**Initial Pose Publishing (for localization):**
- Manual initial pose setting via GUI
- Publishes to `/initialpose` topic
- Automatically activates MOLA localization mode
- Supports arbitrary 6-DOF pose (X, Y, Z, Yaw, Pitch, Roll)

**Planar Motion Control (Runtime Toggle):**
- Runtime toggle of planar motion constraint via GUI buttons
- Enable for flat ground robots (constrains Z, pitch, roll to zero)
- Disable for 3D motion, uneven terrain, or slopes
- Uses MOLA runtime parameter service (`/mola_runtime_param_set`)
- Works with both StateEstimationSmoother and StateEstimationSimple
- Changes take effect immediately without restarting MOLA

**MOLA Activation (for localization):**
- One-click MOLA activation from idle state
- Automatic configuration for localization mode
- Sets `active: true`, `mapping_enabled: false`, `generate_simplemap: false`
- Service-based parameter updates

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimator/pose` | `nav_msgs/Odometry` | Robot pose for trajectory plotting |
| `/mola_diagnostics/lidar_odom/status` | `std_msgs/String` | ICP quality diagnostics |

#### Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Best Effort | Initial pose for localization |

#### Service Clients

| Service | Type | Description |
|---------|------|-------------|
| `/mola_runtime_param_set` | `mola_msgs/MolaRuntimeParamSet` | Runtime parameter modification |

#### GUI Interface

The GUI window provides three main sections:

**1. Initial Pose Section (for localization):**
- X position input (meters)
- Y position input (meters)
- Z position input (meters)
- Yaw orientation input (degrees)
- "Publish Initial Pose & Activate MOLA" button
- Status feedback label

**2. Motion Constraints Section:**
- Real-time planar mode status display
- "Enable Planar Mode" button
- "Disable Planar Mode" button
- Auto-enables planar mode at startup

**3. Configuration Info Panel:**
- Topic information display
- Frame ID settings
- QoS configuration
- Planar mode explanation

**4. Trajectory Plots (6 graphs):**
- XY trajectory (top-down view)
- XZ + YZ trajectory (side views)
- ICP quality over time
- Roll, Pitch, Yaw over time

#### Usage in Localization Workflow

**Launched by:** `mola_localize_launch.py`

**Workflow:**
1. Launch starts with MOLA inactive
2. GUI opens automatically
3. Move robot to known location in map
4. Enter robot's current pose in GUI:
   - X, Y, Z coordinates (meters)
   - Yaw orientation (degrees, 0° = +X axis)
5. Click "Publish Initial Pose & Activate MOLA"
6. GUI automatically:
   - Publishes pose to `/initialpose`
   - Activates MOLA via service
   - Disables mapping mode
   - Disables simplemap generation
7. MOLA begins localizing from specified pose
8. Watch real-time trajectory and ICP quality
9. Toggle planar mode as needed

#### Key Functions

**publish_initial_pose(x, y, z, yaw_degrees):**
- Converts yaw from degrees to quaternion
- Sets covariance matrix (0.25 m², 0.06 rad²)
- Publishes to `/initialpose` with `map` frame_id
- Calls `activate_mola()` automatically

**activate_mola():**
- Calls `/mola_runtime_param_set` service three times:
  1. Sets `active: true` - Wakes MOLA from idle
  2. Sets `mapping_enabled: false` - Localization mode
  3. Sets `generate_simplemap: false` - No map saving
- Returns success status

**set_enforce_planar_motion(enable: bool):**
- Updates state estimator constraint via `/mola_runtime_param_set` service
- Sends YAML parameter update to the state estimator module
- Tries StateEstimationSimple first, then StateEstimationSmoother
- Uses full module instance name format: `classname:label`
- Called by GUI buttons ("Enable Planar Mode" / "Disable Planar Mode")
- Returns success/failure status with console feedback

#### Technical Details

**Coordinate Frames:**
- Initial pose uses `map` frame
- Covariance: diagonal for uncorrelated pose uncertainty
- Default covariance:
  - Position: 0.25 m² (0.5 m std dev)
  - Orientation: 0.06 rad² (~14° std dev)

**Threading:**
- ROS spinning in background daemon thread
- Matplotlib animation in main thread
- GUI updates at 10 Hz (100 ms interval)
- Non-blocking design

**ICP Quality Parsing:**
- Extracts from diagnostics string via regex: `"icp_quality: 0.XX"`
- Plots historical quality over time
- Useful for monitoring localization performance
- Green = good quality, Red = poor quality

---

### Launch File Details

### mola_slam_launch.py - Complete Code

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Point cloud filter
    filterpass = Node(
        package='mola_bringup',
        executable='filterpass.py',
        name='filterpass',
        output='screen'
    )

    # MOLA SLAM
    slam_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', "mola_lidar_odometry",
             'ros2-lidar-odometry-katana.launch.py',
             'lidar_topic_name:=/livox/lidar_filtered',  # Uses filtered cloud
             'imu_topic_name:=/imu',
             'use_rviz:=true',
             'use_mola_gui:=true',
             'start_mapping_enabled:=true',               # Mapping enabled
             'start_active:=true'],                       # Starts immediately
        output='screen'
    )
    
    # Trajectory plotter
    plot_node = Node(
        package='mola_bringup',
        executable='plot_lidar_trajectory.py',
        name='plot_lidar_trajectory',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(filterpass)
    ld.add_action(slam_cmd)
    ld.add_action(plot_node)

    return ld
```

**What it does:**
1. Launches filterpass → cleans `/livox/lidar` → outputs `/livox/lidar_filtered`
2. Launches MOLA in mapping mode with filtered cloud
3. Launches trajectory GUI for visualization

**Outputs:**
- `final_map.simplemap` (current directory)
- `estimated_trajectory.tum` (current directory)

---

### mola_localize_launch.py - Complete Code

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # ⚠️ UPDATE THESE PATHS TO YOUR MAPS ⚠️
    mm_map = '/home/katana/Desktop/array/mola_ws/map/FIBO.mm'
    simple_map = '/home/katana/Desktop/array/mola_ws/map/FIBO.simplemap'

    # Point cloud filter
    filterpass = Node(
        package='mola_bringup',
        executable='filterpass.py',
        name='filterpass',
        output='screen'
    )

    # MOLA Localization
    localize_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', "mola_lidar_odometry",
             'ros2-lidar-odometry-localize-katana.launch.py',
             'lidar_topic_name:=/livox/lidar_filtered',  # Uses filtered cloud
             'imu_topic_name:=/imu',
             'use_rviz:=true',
             'use_mola_gui:=true',
             'enable_mapping:=false',                     # Mapping disabled
             'start_active:=false',                       # Inactive until pose given
             f"mola_initial_map_mm_file:={mm_map}",
             f"mola_initial_map_sm_file:={simple_map}"],
        output='screen'
    )
    
    # Trajectory plotter with initial pose GUI
    plot_node = Node(
        package='mola_bringup',
        executable='plot_lidar_trajectory.py',
        name='plot_lidar_trajectory',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(filterpass)
    ld.add_action(localize_cmd)
    ld.add_action(plot_node)

    return ld
```

**What it does:**
1. Launches filterpass → cleans point cloud
2. Loads pre-built maps from specified paths
3. Launches MOLA in localization mode (INACTIVE)
4. Launches trajectory GUI with initial pose controls
5. Waits for user to provide initial pose via GUI
6. User clicks button → MOLA activates and localizes

**Critical:** You MUST update `mm_map` and `simple_map` paths before using!

**Outputs:**
- No files generated (localization only)

---

### Customization Guide

#### How to Modify Filtering

Edit either launch file and change filterpass parameters:

```python
filterpass = Node(
    package='mola_bringup',
    executable='filterpass.py',
    name='filterpass',
    parameters=[{
        'input_topic': '/livox/lidar',
        'output_topic': '/livox/lidar_filtered',
        'angle_deg': 15.0,           # Custom angle
        'keep_within': False,         # Remove inside cone
        'min_intensity': 25.0,        # Custom intensity
        'max_intensity': 245.0
    }],
    output='screen'
)
```

Rebuild after editing:
```bash
colcon build --packages-select mola_bringup --symlink-install
source install/setup.bash
```

#### How to Change Map Paths

Before each localization session, edit `mola_localize_launch.py`:

```bash
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# Lines 11-12:
mm_map = '/home/YOUR_USERNAME/mola_ws/map/YOUR_MAP.mm'
simple_map = '/home/YOUR_USERNAME/mola_ws/map/YOUR_MAP.simplemap'
```

**Tips:**
- Use absolute paths (not `~/`)
- Verify files exist: `ls -lh /path/to/map.mm`
- Both `.mm` and `.simplemap` should be provided
- Metric map (.mm) is used for localization (faster)
- Simplemap is used as fallback if .mm fails

#### How to Change Topics

If your sensors use different topics, edit both launch files:

```python
slam_cmd = ExecuteProcess(
    cmd=['ros2', 'launch', "mola_lidar_odometry",
         'ros2-lidar-odometry-katana.launch.py',
         'lidar_topic_name:=/YOUR_LIDAR_TOPIC',      # Change this
         'imu_topic_name:=/YOUR_IMU_TOPIC',          # And this
         ...
```

---

### Package Installation (if not already installed)

#### Install from Source

If mola_bringup is not included in your package:

```bash
cd ~/mola_ws/src/MOLA-SLAM/src

# Create package structure
mkdir -p mola_bringup/mola_bringup
mkdir -p mola_bringup/launch
mkdir -p mola_bringup/resource

# Create files (copy from examples above):
# - mola_bringup/mola_bringup/__init__.py
# - mola_bringup/mola_bringup/filterpass.py
# - mola_bringup/mola_bringup/plot_lidar_trajectory.py
# - mola_bringup/launch/mola_slam_launch.py
# - mola_bringup/launch/mola_localize_launch.py
# - mola_bringup/package.xml
# - mola_bringup/CMakeLists.txt
# - mola_bringup/resource/mola_bringup (empty file)

# Make executables
chmod +x mola_bringup/mola_bringup/*.py
chmod +x mola_bringup/launch/*.py

# Build
cd ~/mola_ws
colcon build --packages-select mola_bringup --symlink-install
source install/setup.bash

# Verify
ros2 pkg list | grep mola_bringup
ros2 launch mola_bringup --show-args
```

---

### Troubleshooting mola_bringup

#### Issue 1: No Points After Filtering

**Problem:** FilterPass outputs zero points

**Solutions:**
```bash
# Check input topic
ros2 topic hz /livox/lidar

# Check if any points exist
ros2 topic echo /livox/lidar --no-arr | head -n 50

# Modify launch file to relax filtering:
parameters=[{
    'angle_deg': 180.0,      # Very wide
    'min_intensity': 0.0,     # Accept all
    'max_intensity': 255.0
}]
```

#### Issue 2: GUI Won't Open

**Problem:** plot_lidar_trajectory doesn't show window

**Solutions:**
```bash
# Install dependencies
sudo apt install python3-tk python3-matplotlib

# Check DISPLAY variable
echo $DISPLAY

# If SSH, use X11 forwarding
ssh -X user@robot

# Test matplotlib
python3 -c "import matplotlib.pyplot as plt; plt.plot([1,2,3]); plt.show()"
```

#### Issue 3: MOLA Won't Activate

**Problem:** Clicking GUI button doesn't activate MOLA

**Check:**
```bash
# Is MOLA running?
ros2 node list | grep mola

# Is service available?
ros2 service list | grep mola_runtime_param_set

# Check MOLA launch is from official package (not custom node)
# mola_localize_launch.py must use official ros2-lidar-odometry-localize-katana.launch.py
```

#### Issue 4: Maps Not Loading

**Problem:** "Map file not found" errors

**Check:**
```bash
# Verify paths in launch file
nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py

# Check files exist
ls -lh /home/YOUR_USERNAME/mola_ws/map/*.mm
ls -lh /home/YOUR_USERNAME/mola_ws/map/*.simplemap

# Use absolute paths (not ~/)
# ✓ Good: '/home/username/mola_ws/map/map.mm'
# ✗ Bad:  '~/mola_ws/map/map.mm'
```

#### Issue 5: Wrong Filtered Topic

**Problem:** MOLA not receiving data

**Check:**
```bash
# Verify filtered topic exists
ros2 topic list | grep filtered

# Check data flowing
ros2 topic hz /livox/lidar_filtered

# Verify MOLA is subscribed to filtered topic
ros2 topic info /livox/lidar_filtered
```

---

## External Resources

### Official Documentation

- **MOLA Documentation:** https://docs.mola-slam.org/latest/
- **GitHub Repository:** https://github.com/MOLAorg/mola
- **ROS2 API Reference:** https://docs.mola-slam.org/latest/ros2api.html
- **mp2p_icp Library:** https://github.com/MOLAorg/mp2p_icp
- **Tutorial: Building Maps:** https://docs.mola-slam.org/latest/building-maps.html
- **Tutorial: Localization:** https://docs.mola-slam.org/latest/localization.html

### Related Projects

- **MRPT:** https://www.mrpt.org/
- **MRPT ROS Packages:** https://github.com/MRPT/mrpt_ros
- **MRPT Messages:** https://github.com/mrpt-ros-pkg/mrpt_msgs
- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **Livox SDK:** https://github.com/Livox-SDK/livox_ros_driver2

### Academic References

- Blanco-Claraco JL. A flexible framework for accurate LiDAR odometry, map manipulation, and localization. The International Journal of Robotics Research. 2025;0(0). doi:10.1177/02783649251316881
- J.L. Blanco, [A Modular Optimization Framework for Localization and Mapping](https://ingmec.ual.es/~jlblanco/papers/blanco2019mola_rss2019.pdf), in Robotics: Science and Systems (RSS), 2019.

---

## Best Practices

1. **Use mola_bringup for all workflows:**
   ```bash
   # For mapping:
   ros2 launch mola_bringup mola_slam_launch.py
   
   # For localization:
   ros2 launch mola_bringup mola_localize_launch.py
   ```

2. **Always update map paths before localization:**
   ```bash
   nano ~/mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py
   # Update mm_map and simple_map paths
   ```

3. **Always source workspace:**
   ```bash
   source ~/mola_ws/install/setup.bash
   ```

4. **Use absolute paths for maps:**
   ```python
   # Good:
   mm_map = '/home/username/mola_ws/map/map.mm'
   
   # Bad:
   mm_map = '~/mola_ws/map/map.mm'  # ~ doesn't expand in Python
   ```

5. **Organize maps in dedicated directory:**
   ```bash
   mkdir -p ~/mola_ws/map
   # Keep all .mm and .simplemap files here
   ```

6. **Verify filtered data before mapping:**
   ```bash
   ros2 topic hz /livox/lidar_filtered
   ros2 topic echo /livox/lidar_filtered --no-arr | head
   ```

7. **Monitor ICP quality during operation:**
   - Watch trajectory GUI ICP quality plot
   - Good quality: > 0.60 (GICP), > 0.25 (ICP)
   - If consistently low, adjust filtering or SLAM parameters

8. **Set initial pose accurately:**
   - Use known landmarks in map
   - Align robot orientation carefully
   - Small errors OK (MOLA will refine), large errors cause failure

9. **Test filtering configuration:**
   ```bash
   # Before full mapping, test filter:
   ros2 launch mola_bringup mola_slam_launch.py
   # Drive briefly, check if map builds correctly
   # If issues, adjust filterpass parameters
   ```

10. **Convert maps immediately after mapping:**
    ```bash
    # If using service call (recommended):
    ros2 service call /map_save mola_msgs/srv/MapSave \
      "map_path: '/home/katana/mola_ws/map/building_name'"
    # All files created automatically (.simplemap, .mm, .tum)

    # If using automatic save:
    sm2mm -i final_map.simplemap -o output_map.mm \
      --pipeline $(ros2 pkg prefix mp2p_icp)/share/mp2p_icp/demos/sm2mm_pointcloud_voxelize.yaml
    mv *.simplemap *.mm ~/mola_ws/map/
    ```

11. **Backup important maps:**
    ```bash
    cp ~/mola_ws/map/important_map.* ~/mola_ws/map/backups/
    ```

12. **Build order matters:**
    ```bash
    # Always: mrpt_msgs → mrpt_ros → mola → mola_bringup
    ```

13. **Enable planar mode for ground robots:**
    ```bash
    # In trajectory GUI: Click "Enable Planar Mode" button
    # Or set in YAML config: enforce_planar_motion: true
    # Constrains Z, pitch, roll to zero
    # Improves accuracy for flat terrain
    ```

14. **Disable planar mode for 3D motion:**
    ```bash
    # In trajectory GUI: Click "Disable Planar Mode" button
    # Or set in YAML config: enforce_planar_motion: false
    # For robots on slopes, stairs, or flying
    # Can be toggled at runtime without restarting MOLA
    ```

15. **Name maps descriptively:**
    ```bash
    # Good:
    building_a_floor_2.mm
    outdoor_parking_lot.simplemap
    
    # Bad:
    map1.mm
    test.simplemap
    ```

---

## Important Notes

### Build & Dependencies
- **MRPT Built from Source:** This workspace includes MRPT (v2.14.16) and mrpt_msgs (v0.5.0) to build from source
- **GTSAM Built from Source:** GTSAM factor graph library is included in the workspace (do NOT install libgtsam-dev from apt)
- **DO NOT install ros-humble-mrpt-\* from apt:** Installing both source and apt versions will cause conflicts
- **75+ Dependencies Required:** Use the complete dependency install command in the Installation section
- **ROS2 Must Be Sourced:** Run `source /opt/ros/humble/setup.bash` before building
- **Build Order Critical:** Always build: gtsam → mrpt_msgs → mrpt_libs → mola packages → mola_bringup
- **Build Time:** First build takes 30-60 minutes (GTSAM and MRPT compilation is intensive)
- **No rosdep:** All dependencies installed manually via apt (listed in Installation section)

### Package Structure & Paths
- **mola_bringup Location:** `mola_ws/src/MOLA-SLAM/src/mola_bringup/`
- **Launch Files:** `mola_slam_launch.py` (mapping) and `mola_localize_launch.py` (localization)
- **Script Locations:** `mola_bringup/mola_bringup/filterpass.py` and `plot_lidar_trajectory.py`
- **Navigation Packages:** mrpt_navigation (pf_localization, reactivenav2d, map_server) and mrpt_path_planning included for autonomous navigation capabilities

### Launch File Configuration
- **Hardcoded Map Paths:** `mola_localize_launch.py` contains example paths that **MUST be updated**
- **Edit Before Use:** `nano mola_ws/src/MOLA-SLAM/src/mola_bringup/launch/mola_localize_launch.py`
- **Use Absolute Paths:** Not `~/` syntax (Python doesn't expand it)
- **Localization Behavior:** MOLA starts **INACTIVE** and requires initial pose via GUI button
- **Filtered Topic:** Both launch files use `/livox/lidar_filtered` as input to MOLA

### System Behavior
- **Output Files:** Mapping produces `.simplemap` and `.tum` files (no PCD by default)
- **State Estimator:** Recommended for smoother trajectories and better sensor fusion
- **GICP vs ICP:** GICP = higher accuracy (outdoor), ICP = faster (indoor)
- **REP105 Compliance:** Enabled by default for standard ROS2 navigation compatibility
- **Frame Names:** Ensure frame names match your URDF/robot configuration
- **GUI Auto-enables Planar Mode:** plot_lidar_trajectory automatically enables planar motion at startup

---

## Package Contents

This package includes:

```
~/mola_ws/src/
├── MOLA-SLAM/
│   └── src/
│       ├── mola/                          # MOLA metapackage (v2.2.0)
│       │
│       ├── mola_bringup/                  # Custom launch package for SLAM workflows
│       │   ├── mola_bringup/
│       │   │   ├── filterpass.py
│       │   │   └── plot_lidar_trajectory.py
│       │   └── launch/
│       │       ├── mola_slam_launch.py    # FOR MAPPING
│       │       └── mola_localize_launch.py # FOR LOCALIZATION
│       │
│       ├── mola_kernel/                   # Virtual interfaces & data types (v2.2.0)
│       ├── mola_launcher/                 # Module launcher (v2.2.0)
│       ├── mola_viz/                      # Visualization (v2.2.0)
│       ├── mola_yaml/                     # YAML configuration (v2.2.0)
│       ├── mola_msgs/                     # MOLA message types (v2.2.0)
│       ├── mola_common/                   # Common CMake scripts (v0.5.2)
│       ├── mola_metric_maps/              # Metric map types (v2.2.0)
│       ├── mola_pose_list/                # Pose list utilities (v2.2.0)
│       ├── mola_traj_tools/               # Trajectory tools (v2.2.0)
│       ├── mola_relocalization/           # Relocalization module (v2.2.0)
│       ├── mola_bridge_ros2/              # ROS2 bridge (v2.2.0)
│       │
│       ├── mola_lidar_odometry/           # LiDAR odometry (v1.2.1)
│       ├── mola_imu_preintegration/       # IMU preintegration (v1.13.2)
│       ├── mola_state_estimation/         # State estimation metapackage (v1.11.1)
│       │   ├── mola_state_estimation_simple/
│       │   └── mola_state_estimation_smoother/
│       │
│       ├── mp2p_icp/                      # Multi-Primitive ICP library (v2.1.0)
│       │
│       ├── mola_input_euroc_dataset/      # EuRoC dataset input
│       ├── mola_input_kitti_dataset/      # KITTI dataset input
│       ├── mola_input_kitti360_dataset/   # KITTI-360 dataset input
│       ├── mola_input_mulran_dataset/     # MulRan dataset input
│       ├── mola_input_paris_luco_dataset/ # Paris Luco dataset input
│       ├── mola_input_rawlog/             # Rawlog file input
│       │
│       ├── mrpt_libros_bridge/            # MRPT-ROS bridge (v2.14.15)
│       ├── rosbag2rawlog/                 # ROS bag to rawlog converter
│       ├── kitti_metrics_eval/            # KITTI metrics evaluation
│       └── mola_demos/                    # Demo configurations
│
├── gtsam/                             # GTSAM factor graph library (v4.2.0)
│
├── mrpt_msgs-0.5.0/                   # MRPT ROS2 message definitions
│
├── mrpt_navigation/                   # MRPT navigation stack (v2.3.0)
│   ├── mrpt_pf_localization/          # Particle filter localization
│   ├── mrpt_reactivenav2d/            # Reactive 2D navigation
│   ├── mrpt_map_server/               # Map server
│   ├── mrpt_pointcloud_pipeline/      # Point cloud processing
│   ├── mrpt_tps_astar_planner/        # TPS A* path planner
│   ├── mrpt_nav_interfaces/           # Navigation interfaces
│   ├── mrpt_msgs_bridge/              # Message bridge
│   └── mrpt_tutorials/                # Tutorial packages
│
├── mrpt_path_planning/                # MRPT path planning library (v0.2.3)
│
├── mrpt_ros-2.14.16/                  # MRPT ROS2 libraries
│   ├── mrpt_libbase/                  # Core libraries
│   ├── mrpt_libmath/                  # Math libraries
│   ├── mrpt_libposes/                 # Pose representations
│   ├── mrpt_libmaps/                  # Map data structures
│   ├── mrpt_libobs/                   # Observation types
│   ├── mrpt_libslam/                  # SLAM algorithms
│   ├── mrpt_libnav/                   # Navigation
│   ├── mrpt_libopengl/                # OpenGL rendering
│   ├── mrpt_libgui/                   # GUI widgets
│   ├── mrpt_libhwdrivers/             # Hardware drivers
│   └── mrpt_apps/                     # MRPT applications
│
└── pose_cov_ops/                      # Pose covariance operations (v0.4.0)
```

All dependencies are pre-configured and version-locked for compatibility.

**Primary Usage:** The two launch files in `mola_bringup` are your main interface to the system.

**Key Path:** `~/mola_ws/src/MOLA-SLAM/src/mola_bringup/`

---

## Support

**Common Issues:** See [Troubleshooting](#troubleshooting) section above

**Debug Checklist:**
1. ✅ Workspace sourced: `source ~/mola_ws/install/setup.bash`
2. ✅ For localization: Map paths updated in mola_localize_launch.py
3. ✅ Sensor data flowing: `ros2 topic hz /livox/lidar`
4. ✅ Filtered data flowing: `ros2 topic hz /livox/lidar_filtered`
5. ✅ TF tree complete: `ros2 run tf2_tools view_frames`
6. ✅ For localization: Initial pose provided via GUI
7. ✅ For localization: MOLA activated via GUI button
8. ✅ mola_bringup built: `ros2 pkg list | grep mola_bringup`

**Getting Help:**
- Check MOLA official documentation: https://docs.mola-slam.org/latest/
- Review this README's troubleshooting sections
- Verify all steps in Quick Start guide followed correctly

---

## License

This documentation is for MOLA SLAM configuration on ROS2 Humble.  
MOLA SLAM is licensed under BSD-3-Clause. See individual packages for details.

---

**Version:** 1.8
**All packages built from source**

| Package | Version |
|---------|---------|
| MOLA | v2.2.0 |
| GTSAM | v4.2.0 |
| MRPT | v2.14.16 |
| mrpt_msgs | v0.5.0 |
| mola_lidar_odometry | v1.2.1 |
| mp2p_icp | v2.1.0 |
| mrpt_navigation | v2.3.0 |
| mrpt_path_planning | v0.2.3 |
| pose_cov_ops | v0.4.0 |

**Last Updated:** December 2025
**Workspace:** `~/mola_ws`
