# AutoGiro Simulation

ROS2 simulation package for the AutoGiro, a differential-drive autonomous vehicle. Includes Gazebo simulation environments, robot description (URDF/Xacro), Nav2 navigation, SLAM, and hardware launch support.

The primary simulation environment is a model of the Acopian Engineering Center 4th floor.

## Prerequisites

- ROS2 Humble
- Gazebo (Ignition)
- Required ROS2 packages:
  - `robot_state_publisher`
  - `xacro`
  - `joint_state_publisher`
  - `ros_gz_sim`, `ros_gz_bridge`
  - `ros2_control`, `gazebo_ros2_control`
  - `teleop_twist_keyboard`
  - `nav2` (for navigation)
  - `slam_toolbox` (for mapping)

## Project Structure

```
autogiro/
├── config/          # Controller, Nav2, SLAM, joystick, and RViz configs
├── description/     # Robot URDF/Xacro files (chassis, lidar, control)
├── launch/          # Launch files for sim, real robot, nav, SLAM, etc.
├── models/          # Gazebo SDF models (Acopian 4th floor)
├── worlds/          # Gazebo world files (empty, obstacles, Acopian)
└── scripts/         # Helper shell scripts
```

## Simulation Launch Procedure

Each step runs in its own terminal, as each process runs simultaneously.

### 1. Launch AEC Floorplan Simulation

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Navigate to working directory and pull latest changes
cd autogiro_sim
git pull

# Source, build, and launch
source install/setup.bash
colcon build
ros2 launch autogiro launch_floorplan.launch.py
```

**Visualizing LiDAR in Gazebo:** Click the three dots in the top-left corner of the Gazebo window and search "lidar". Click "Visualize LiDAR", expand the menu, and click the topic refresh button. `/scan` should populate the box, and the rays should become visible.

### 2. Run Teleop Keyboard

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```

You can only control the robot when this terminal is focused. Operational instructions are displayed in the terminal.

### 3. Run Robot State Publisher

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch autogiro rsp.launch.py
```

### 4. Launch RViz

```bash
cd ~/ros2_ws
rviz2
```

In the RViz2 Displays panel, click **Add** and choose **RobotModel**. Expand the dropdown until **Description Topic** is visible, then set it to `/robot_description`. For a top-down view, select **TopDownOrtho** from the camera views in the top-right panel.

## Mapping and Localization Workflow

The workflow is: build a map with SLAM, save it to `maps/`, then relaunch using AMCL localization against the saved map.

### Phase A — Build a map

Each command in its own terminal.

```bash
# Terminal 1: sim + robot + sensors
ros2 launch autogiro launch_floorplan.launch.py

# Terminal 2: slam_toolbox in mapping mode
ros2 launch autogiro online_async_launch.py

# Terminal 3: drive the robot around until the map looks complete in RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```

In RViz, add a `Map` display on topic `/map` to see the map fill in.

### Phase B — Save the map

From the package source directory (so the `maps/` folder is picked up as the default output dir):

```bash
cd ~/ros2_ws/autogiro_sim/autogiro
ros2 run autogiro save_map.sh acopian
```

This writes four files into `maps/`:

- `acopian.yaml` + `acopian.pgm` — nav2/AMCL format (occupancy grid)
- `acopian.posegraph` + `acopian.data` — slam_toolbox format (for resuming mapping or slam_toolbox localization mode)

Rebuild so the installed share directory picks up the new map:

```bash
cd ~/ros2_ws
colcon build --packages-select autogiro
source install/setup.bash
```

### Phase C — Launch with localization

```bash
# Terminal 1: sim + robot + sensors
ros2 launch autogiro launch_floorplan.launch.py

# Terminal 2: AMCL + map_server (defaults to maps/acopian.yaml)
ros2 launch autogiro localization_launch.py
```

To use a different saved map:

```bash
ros2 launch autogiro localization_launch.py map:=/absolute/path/to/other.yaml
```

AMCL's initial pose is seeded from `config/nav2_params.yaml` to match the Gazebo spawn location. Drive the robot a short distance for the particle cloud to converge, or override via the `2D Pose Estimate` tool in RViz.

### Phase D — Optional: navigation

After Phase C, launch the nav2 stack and send goals via RViz's `2D Goal Pose`:

```bash
ros2 launch autogiro navigation_launch.py
```

## Other Launch Configurations

| Launch File | Description |
|---|---|
| `launch_sim.launch.py` | Basic simulation in an empty world |
| `launch_floorplan.launch.py` | Acopian 4th floor simulation |
| `launch_robot.launch.py` | Real robot hardware launch |
| `navigation_launch.py` | Nav2 navigation stack |
| `localization_launch.py` | AMCL localization with map server |
| `online_async_launch.py` | SLAM Toolbox for mapping |
| `joystick.launch.py` | Joystick teleop input |
| `camera.launch.py` | USB camera node |
| `rplidar.launch.py` | RPLidar hardware driver |

## Robot Specifications

- **Dimensions:** 0.93m x 0.745m x 1.75m
- **Mass:** 68 kg
- **Drive:** Differential drive, wheel radius 0.161m, wheel separation 0.635m
- **Sensors:** GPU LiDAR (360 deg, 0.1-20m range, 10 Hz)
- **Control:** ros2_control with diff_drive_controller (50 Hz)

## Key Configuration Files

| File | Purpose |
|---|---|
| `config/my_controllers.yaml` | Diff drive controller parameters |
| `config/twist_mux.yaml` | Command velocity priority mux (joystick > tracker > nav) |
| `config/nav2_params.yaml` | Nav2 stack parameters |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox parameters |
