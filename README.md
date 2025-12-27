# 🤖 Autonomous Robot Apartment Patrol

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](http://classic.gazebosim.org/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-green)](https://www.python.org/)

Fully autonomous mobile robot patrol system for apartment environments using ROS 2 Humble, Gazebo Classic, SLAM, and Nav2 Navigation Stack.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

---

## 🎯 Overview

This project implements an autonomous mobile robot capable of patrolling a 2+1 apartment environment. The robot uses SLAM for mapping, AMCL for localization, and Nav2 for autonomous navigation through predefined waypoints.

### What It Does

- **Maps** the apartment environment using SLAM Toolbox
- **Localizes** itself using AMCL (Adaptive Monte Carlo Localization)
- **Navigates** autonomously between 5 predefined waypoints:
  - 🏠 Salon (Living Room)
  - 🍳 Mutfak (Kitchen)
  - 🛏️ Yatak Odası 1 (Bedroom 1)
  - 🛏️ Yatak Odası 2 (Bedroom 2)
  - 🚪 Koridor (Hallway)
- **Patrols** continuously in an infinite loop

---

## ✨ Features

### Core Functionality
- ✅ **Autonomous Navigation** using Nav2 Stack
- ✅ **SLAM Mapping** with SLAM Toolbox
- ✅ **Differential Drive Robot** with realistic physics
- ✅ **Lidar-based** obstacle detection and avoidance
- ✅ **Custom Apartment World** in Gazebo Classic
- ✅ **Simulation Time Synchronization** for accurate navigation
- ✅ **Real-time Status Feedback** with detailed logging

### Sensors
- 📡 **Lidar** (360° range sensor)
- 📷 **RGB Camera**
- 🧭 **Odometry** (wheel encoders)

### Robot Specifications
- **Type:** Differential Drive Mobile Robot
- **Wheel Separation:** 0.297m
- **Wheel Diameter:** 0.066m
- **Max Linear Velocity:** 0.26 m/s
- **Max Angular Velocity:** 1.0 rad/s
- **Robot Radius:** ~0.20m

---

## 💻 System Requirements

### Software
- **OS:** Ubuntu 22.04 LTS (Jammy)
- **ROS 2:** Humble Hawksbill
- **Gazebo:** Classic 11
- **Python:** 3.10+

### ROS 2 Packages
```bash
ros-humble-gazebo-ros-pkgs
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox
ros-humble-turtlebot3*
```

### Hardware (Recommended for Simulation)
- **CPU:** Intel i5 or AMD Ryzen 5 (4+ cores)
- **RAM:** 8GB minimum, 16GB recommended
- **GPU:** Integrated graphics sufficient, dedicated GPU recommended
- **Storage:** 20GB free space

---

## 🚀 Installation

### 1. Prerequisites

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Install Dependencies

```bash
# Install Gazebo Classic and ROS 2 packages
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions
```

### 3. Clone and Build

```bash
# Create workspace
mkdir -p ~/yeni_ws/src
cd ~/yeni_ws/src

# Clone this repository
git clone <your-repo-url> Autonomous-Robot-Obstacle-Avoidance-with-ROS2

# Build
cd ~/yeni_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_robot
source install/setup.bash
```

---

## 📁 Project Structure

```
src/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/
├── src/auto_robot/
│   ├── config/
│   │   └── params_narrow.yaml          # Nav2 parameters (narrow doorways)
│   ├── description/
│   │   ├── camera.xacro                # Camera sensor URDF
│   │   ├── gazebo_control.xacro        # Differential drive plugin
│   │   ├── lidar.xacro                 # Lidar sensor URDF
│   │   └── robot_core.xacro            # Robot URDF core
│   ├── launch/
│   │   └── launch_sim.launch.py        # Main Gazebo launch file
│   └── worlds/
│       └── my_home.sdf                 # Custom apartment world
├── patrol.py                            # Autonomous patrol script
└── README.md                            # This file

maps/
├── my_home_map.yaml                     # Map metadata
└── my_home_map.pgm                      # Occupancy grid image
```

---

## 🎮 Usage

### Quick Start (3 Terminals Required)

#### Terminal 1: Launch Gazebo Simulation

```bash
cd ~/yeni_ws
source install/setup.bash
ros2 launch auto_robot launch_sim.launch.py
```

**Expected:** Gazebo opens with robot spawned in apartment

#### Terminal 2: Launch Nav2 Navigation Stack

```bash
cd ~/yeni_ws
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=$(pwd)/maps/my_home_map.yaml
```

**Wait ~20 seconds for Nav2 to activate**

#### Terminal 3: Run Autonomous Patrol

```bash
cd ~/yeni_ws
source install/setup.bash
python3 src/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/patrol.py \
    --ros-args -p use_sim_time:=true
```

**Expected Output:**
```
🕐 Initializing BasicNavigator with use_sim_time=True...
✓ use_sim_time parameter: True

======================================================================
🏠 AUTONOMOUS APARTMENT PATROL
======================================================================
📍 Waypoints: 5 locations
🤖 Robot will visit each room sequentially
======================================================================

⏳ Waiting for Nav2 to become active...
✅ Nav2 is ready!

🕐 Initial pose timestamp: 60.700000000
📍 Initial pose set: Koridor (x=-2.0, y=0.0)

[1/5] 🎯 Going to: Salon (Living Room)
             Coordinates: (x=-4.5, y=2.0, yaw=0.0)
             🕐 Goal timestamp: 62.500000000
             ✅ Arrived at Salon (Living Room)!
```

---

### Optional: Visualize with RViz

```bash
# Terminal 4
cd ~/yeni_ws
source install/setup.bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Add displays:**
- `Map` → Topic: `/map`
- `Costmap` → Topic: `/global_costmap/costmap`
- `Costmap` → Topic: `/local_costmap/costmap`
- `LaserScan` → Topic: `/scan`
- `Path` → Topic: `/plan`

---

## 🔧 Configuration

### Waypoint Coordinates

Edit waypoints in `patrol.py` (line 59-65):

```python
waypoints = [
    ("Salon (Living Room)", -4.5, 2.0, 0.0),
    ("Mutfak (Kitchen)", -4.5, -3.0, 0.0),
    ("Yatak Odasi 1 (Bedroom 1)", 4.5, 2.0, 0.0),
    ("Yatak Odasi 2 (Bedroom 2)", 4.5, -3.0, 0.0),
    ("Koridor (Hallway - Home)", -2.0, 0.0, 0.0),
]
```

**Format:** `(name, x, y, yaw)`
- **x, y:** Coordinates in map frame (meters)
- **yaw:** Orientation in radians

---

### Nav2 Parameters

**Default:** Uses Nav2 default parameters

**Custom (Narrow Doorways):**
```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=$(pwd)/maps/my_home_map.yaml \
    params_file:=$(pwd)/install/auto_robot/share/auto_robot/config/params_narrow.yaml
```

**Key parameters in `params_narrow.yaml`:**
- `inflation_radius: 0.25` (minimal for tight spaces)
- `cost_scaling_factor: 10.0` (less conservative)
- `robot_radius: 0.17`

---

### Robot Velocities

Edit in `description/gazebo_control.xacro`:

```xml
<max_vel_x>0.26</max_vel_x>
<max_vel_theta>1.0</max_vel_theta>
<acc_lim_x>2.5</acc_lim_x>
<acc_lim_theta>3.2</acc_lim_theta>
```

---

## 🗺️ Creating a New Map

If you modify the apartment world or want a fresh map:

### 1. Launch Gazebo + SLAM

```bash
# Terminal 1
cd ~/yeni_ws && source install/setup.bash
ros2 launch auto_robot launch_sim.launch.py

# Terminal 2
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=True
```

### 2. Manual Teleoperation

```bash
# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel
```

**Drive through ALL rooms carefully**

### 3. Save the Map

```bash
# Terminal 4
cd ~/yeni_ws
ros2 run nav2_map_server map_saver_cli \
    -f maps/my_home_map
```

---

## 🐛 Troubleshooting

### Issue: Robot not moving

**Symptoms:** Nav2 plans path but robot doesn't move

**Solutions:**
1. Check `use_sim_time` parameter:
   ```bash
   python3 patrol.py --ros-args -p use_sim_time:=true
   ```

2. Verify `/cmd_vel` topic:
   ```bash
   ros2 topic hz /cmd_vel
   ```

3. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

---

### Issue: "Frame does not exist" errors

**Symptoms:** Nav2 complains about missing `odom` or `map` frames

**Solutions:**
1. **Ensure Gazebo is running BEFORE Nav2**
2. Verify robot spawned:
   ```bash
   ros2 topic echo /odom --once
   ```

3. Check TF:
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```

---

### Issue: Nav2 won't activate

**Symptoms:** Patrol script stuck at "Waiting for Nav2..."

**Solutions:**
1. Check Nav2 node status:
   ```bash
   ros2 node list | grep -E 'amcl|bt_navigator'
   ```

2. Check lifecycle state:
   ```bash
   ros2 lifecycle get /bt_navigator
   ```

   Should return: `active [3]`

3. Restart Nav2 with verbose logging:
   ```bash
   ros2 launch nav2_bringup bringup_launch.py \
       use_sim_time:=True \
       map:=$(pwd)/maps/my_home_map.yaml \
       log_level:=debug
   ```

---

### Issue: Gazebo crashes on launch

**Symptoms:** `gzserver` dies with exit code 255

**Solutions:**
1. Clear Gazebo cache:
   ```bash
   rm -rf ~/.gazebo/models/auto_car
   ```

2. Check SDF file:
   ```bash
   gz sdf -k src/Autonomous-Robot-Obstacle-Avoidance-with-ROS2/src/auto_robot/worlds/my_home.sdf
   ```

3. Reinstall Gazebo:
   ```bash
   sudo apt reinstall gazebo libgazebo11
   ```

---

## 📊 Performance Metrics

### Tested Performance
- **Navigation Success Rate:** ~80% (depends on waypoint placement)
- **Average Speed:** 0.15-0.20 m/s
- **Path Replanning:** ~1-2 times per waypoint
- **Cycle Time:** ~3-5 minutes (5 waypoints)
- **CPU Usage:** 30-50% (4-core Intel i5)
- **RAM Usage:** ~2-3GB

---

## 🔬 Technical Details

### Coordinate System
- **Origin:** Robot spawn location
- **X-axis:** Forward (red)
- **Y-axis:** Left (green)
- **Z-axis:** Up (blue)

### Map Details
- **Resolution:** 0.05 m/pixel
- **Size:** 279 x 198 cells
- **Origin:** [-7.0, -5.0, 0.0]
- **Occupied Threshold:** 0.65
- **Free Threshold:** 0.25

### TF Tree
```
map
 └─ odom
     └─ base_footprint
         └─ base_link
             ├─ laser_frame
             ├─ camera_link
             │   └─ camera_link_optical
             ├─ left_wheel
             ├─ right_wheel
             └─ caster_wheel
```

---

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 👥 Authors

- **Your Name** - Initial work

---

## 🙏 Acknowledgments

- ROS 2 Navigation Team
- Gazebo Development Team
- SLAM Toolbox Contributors
- TurtleBot3 Team (for reference implementations)

---

## 📚 References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Classic Documentation](http://classic.gazebosim.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

## 🎓 Learn More

### Tutorials Used
- [Nav2 Getting Started](https://navigation.ros.org/getting_started/index.html)
- [SLAM Toolbox Tutorial](https://github.com/SteveMacenski/slam_toolbox#tutorials)
- [Gazebo ROS 2 Integration](http://classic.gazebosim.org/tutorials?tut=ros2_overview)

---

**Made with ❤️ using ROS 2 Humble**