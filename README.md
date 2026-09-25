# ROS 2 Autonomous Navigation Assignment - Mukhtar Ahmed

Autonomous navigation implementation for a differential-drive robot testbed using **ROS 2 Humble**, **Gazebo Classic**, and **Nav2**.

## 1. Project Overview

This project implements an end-to-end autonomous navigation system for a simulated differential-drive robot in a custom Gazebo environment.

The system includes:

* Map loading using `nav2_map_server`
* AMCL-based localization
* Global path planning using `NavFn`
* Local trajectory control using `DWB`
* Nav2 costmaps and recovery behaviors
* Gazebo simulation with simulated LiDAR, IMU, and odometry
* RViz2 visualization and navigation testing

The original repository contained several configuration and simulation issues. These were identified, corrected, and tested during the implementation.

---

## 2. Package Structure

```text
assignment_ws/
└── src/
    └── level01_ros_assignment/
        ├── testbed_description/
        │   ├── meshes/
        │   ├── urdf/
        │   │   ├── testbed.urdf
        │   │   └── testbed.gazebo
        │   └── launch/
        │       └── robot_description.launch.py
        │
        ├── testbed_gazebo/
        │   ├── models/
        │   ├── worlds/
        │   │   └── testbed_playground.world
        │   └── launch/
        │       ├── spawn_playground.launch.py
        │       └── spawn_testbed.launch.py
        │
        ├── testbed_bringup/
        │   ├── launch/
        │   │   └── testbed_full_bringup.launch.py
        │   └── maps/
        │       ├── testbed_world.yaml
        │       └── testbed_world.pgm
        │
        └── testbed_navigation/
            ├── CMakeLists.txt
            ├── package.xml
            ├── config/
            │   ├── amcl.yaml
            │   ├── nav2_params.yaml
            │   ├── testbed_world.yaml
            │   ├── testbed_world.pgm
            │   └── nav2_view.rviz
            └── launch/
                ├── map_server.launch.py
                ├── localization.launch.py
                ├── navigation.launch.py
                └── rviz.launch.py
```

---

# 3. Bugs Identified and Fixed

## 3.1 CMake Syntax Error

**File:** `testbed_navigation/CMakeLists.txt`

### Problem

The workspace failed during `colcon build`.

### Cause

The `ament_package()` macro was missing its closing parenthesis.

### Fix

```cmake
ament_package()
```

---

## 3.2 Gazebo World Was Not Loaded

**File:** `testbed_gazebo/launch/spawn_playground.launch.py`

### Problem

Gazebo opened the default empty world instead of the provided playground world.

### Cause

The `world` launch argument was declared but was not forwarded to the included Gazebo launch file.

### Fix

The `world` argument was passed through the `IncludeLaunchDescription` action:

```python
launch_arguments={
    'world': LaunchConfiguration('world')
}.items()
```

---

## 3.3 Incorrect Map Image Path

**File:** `testbed_world.yaml`

### Problem

The map server failed because the map image could not be found.

### Cause

The YAML file referenced:

```yaml
image: wrong_path_testbed_world.pgm
```

### Fix

Changed it to:

```yaml
image: testbed_world.pgm
```

---

## 3.4 Malformed Gazebo XML

**File:** `testbed_description/urdf/testbed.gazebo`

### Problem

Robot description parsing produced XML/URDF errors during spawning.

### Cause

An extra `>` was present at the end of the XML tag.

### Fix

Changed:

```xml
<initial_orientation_as_reference>false</initial_orientation_as_reference>>
```

to:

```xml
<initial_orientation_as_reference>false</initial_orientation_as_reference>
```

---

## 3.5 Missing Runtime Dependencies

**File:** `testbed_description/package.xml`

### Problem

Robot state and joint state publishing could fail on a clean environment.

### Cause

Required runtime dependencies were not declared.

### Fix

Added:

```xml
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
```

The required packages were also installed on the development system.

---

## 3.6 Differential-Drive Simulation Physics

**File:** `testbed_description/urdf/testbed.gazebo`

### Problem

The robot's odometry indicated that it had moved farther than the physical robot in Gazebo.

### Cause

The simulated wheel and caster friction settings caused excessive wheel slip and resistance.

The differential-drive plugin also required higher wheel torque.

### Fix

Drive-wheel friction was increased:

```xml
<mu1>1.0</mu1>
<mu2>1.0</mu2>
```

Caster friction was reduced:

```xml
<mu1>0.001</mu1>
<mu2>0.001</mu2>
```

Wheel torque was increased:

```xml
<max_wheel_torque>50.0</max_wheel_torque>
```

These changes improved the agreement between the simulated robot movement and `/odom`.

---

## 3.7 LiDAR False Obstacle Ring

**File:** `testbed_navigation/config/nav2_params.yaml`

### Problem

A circular ring of obstacles appeared around the robot in RViz.

### Cause

The simulated LiDAR has a maximum range of approximately `1.5 m`.

When no obstacle was detected, maximum-range readings could be interpreted by the costmap as obstacles when the obstacle range was configured at the same value.

### Fix

The costmap configuration was adjusted:

```yaml
obstacle_max_range: 1.35
raytrace_max_range: 2.0
inf_is_valid: true
```

This prevents maximum-range readings from being incorrectly treated as solid obstacles.

---

## 3.8 Global Costmap Obstacle Persistence

**File:** `testbed_navigation/config/nav2_params.yaml`

### Problem

Laser scans during robot rotation could leave obstacle traces in the global costmap and interfere with path planning.

### Cause

The global costmap was processing real-time laser observations directly.

### Fix

The global costmap was configured primarily with:

```yaml
plugins:
  - static_layer
  - inflation_layer
```

Real-time obstacle detection was handled by the local costmap.

---

## 3.9 Simulation Time Synchronization

### Problem

RViz and Nav2 produced timestamp-related message filter warnings.

### Cause

Different nodes were not consistently using Gazebo simulation time.

### Fix

Simulation time was enabled for the relevant nodes:

```yaml
use_sim_time: true
```

---

# 4. Technical Implementation

## 4.1 TF Tree

The robot uses the following transform structure:

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── lidar_link_1
            ├── imu_link_1
            ├── left_wheel_1
            ├── right_wheel_1
            └── caster links
```

The main transforms are:

* `map → odom` — published by AMCL for global localization.
* `odom → base_footprint` — published by the differential-drive Gazebo plugin.
* `base_footprint → base_link` — published by `robot_state_publisher`.

The TF tree was verified using `view_frames` and `tf2_echo`.

---

## 4.2 Map Server

The map is loaded using `nav2_map_server` and managed through a lifecycle manager.

The map configuration is:

```text
Resolution: 0.05 m/pixel
Dimensions: 405 × 400 pixels
Origin: [-10.2, -9.94, 0.0]
Frame: map
```

The map server can be launched independently:

```bash
ros2 launch testbed_navigation map_server.launch.py
```

---

## 4.3 AMCL Localization

Localization is implemented using `nav2_amcl`.

The main configuration uses:

```text
Base frame: base_footprint
Odometry frame: odom
Global frame: map
Minimum particles: 500
Maximum particles: 2000
LiDAR maximum range: 1.5 m
```

The initial robot pose is provided through RViz using **2D Pose Estimate**.

> The 2D Pose Estimate tool provides AMCL with the robot's estimated pose. It does not physically teleport the robot inside Gazebo.

---

## 4.4 Nav2 Navigation

The navigation stack consists of:

### Global Planner

```text
nav2_navfn_planner/NavfnPlanner
```

Used for grid-based global path planning.

### Local Controller

```text
dwb_core::DWBLocalPlanner
```

DWB evaluates possible local trajectories using critics including:

* `RotateToGoal`
* `BaseObstacle`
* `PathAlign`
* `GoalAlign`
* `PathDist`
* `GoalDist`

### Behavior Tree Navigator

The Behavior Tree Navigator coordinates navigation behaviors and recovery actions such as:

* Spin
* BackUp
* Wait
* ClearCostmap

### Costmaps

The system uses layered 2D costmaps consisting of:

* Static layer
* Obstacle layer
* Inflation layer

The costmap configuration was adjusted according to the robot footprint and simulated LiDAR characteristics.

---

# 5. Build and Installation

## 5.1 Create the Workspace

```bash
mkdir -p ~/assignment_ws/src
cd ~/assignment_ws/src
```

Clone the repository:

```bash
git clone https://github.com/Cyber-M4/level01_ros_assignment.git
```

---

## 5.2 Install Dependencies

```bash
cd ~/assignment_ws

rosdep update

rosdep install -y -r -q \
  --from-paths src \
  --ignore-src \
  --rosdistro humble
```

Install the additional required packages:

```bash
sudo apt install \
  ros-humble-joint-state-publisher \
  ros-humble-nav2-bringup \
  -y
```

---

## 5.3 Build the Workspace

```bash
cd ~/assignment_ws
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/assignment_ws/install/setup.bash
```

---

# 6. Running the Simulation

Launch the following components in separate terminals.

## Terminal 1 — Gazebo Playground

```bash
source /usr/share/gazebo/setup.sh 2>/dev/null || source /usr/share/gazebo-11/setup.sh 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_gazebo spawn_playground.launch.py
```

---

## Terminal 2 — Robot Description

```bash
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_description robot_description.launch.py
```

---

## Terminal 3 — Spawn Robot

```bash
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_gazebo spawn_testbed.launch.py
```

---

## Terminal 4 — Map Server

```bash
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_navigation map_server.launch.py
```

---

## Terminal 5 — AMCL Localization

```bash
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_navigation localization.launch.py
```

---

## Terminal 6 — Nav2 Navigation

```bash
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_navigation navigation.launch.py use_sim_time:=true
```

---

## Terminal 7 — RViz

The project includes a custom RViz launch file that loads the saved navigation configuration automatically.

```bash
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash

ros2 launch testbed_navigation rviz.launch.py
```

The RViz configuration is stored at:

```text
testbed_navigation/config/nav2_view.rviz
```

---

# 7. Operating the Robot

## Step 1 — Set the Initial Pose

In RViz:

1. Select **2D Pose Estimate**.
2. Click at the robot's actual position in Gazebo.
3. Drag the arrow in the direction the robot is facing.
4. AMCL uses this estimate to initialize localization.

---

## Step 2 — Send a Navigation Goal

1. Select **Nav2 Goal** or **2D Goal Pose**.
2. Click an unoccupied location on the map.
3. Nav2 calculates a global path.
4. DWB generates local velocity commands.
5. The robot moves toward the goal in Gazebo.

A successful navigation run should produce a message similar to:

```text
[INFO] [bt_navigator]: Navigation succeeded
```

---

# 8. Verification

The final system was tested to verify the main navigation components.

### Gazebo

Verified:

* Playground world loading
* Robot spawning
* Differential-drive motion
* Contact physics
* Simulated sensors

### Sensor Topics

Confirmed active publishing on:

```text
/scan
/odom
/imu
/clock
```

### TF

Verified connectivity from:

```text
map
└── odom
    └── base_footprint
        └── base_link
```

and the corresponding sensor and wheel frames.

### Map

Verified `/map` publication with:

```text
Resolution: 0.05 m/pixel
Size: 405 × 400
Frame: map
```

### Localization

AMCL successfully converges after providing the initial pose through RViz and maintains localization while the robot moves.

### Navigation

Nav2 successfully:

* Generates global paths.
* Executes local trajectory control.
* Uses costmaps for obstacle handling.
* Publishes velocity commands.
* Reaches commanded navigation goals.

---

# 9. Results and Demonstration

The final system was successfully tested in Gazebo Classic and RViz2. The robot was able to localize itself using AMCL, generate global paths using Nav2, execute local trajectory control, and navigate to commanded goal positions while interacting with the simulated environment. The following demonstration video and screenshots show the complete navigation pipeline, including the Gazebo environment, RViz visualization, robot localization, path planning, and successful goal execution.

## Demonstration Video

[![ROS 2 Autonomous Navigation Assignment](images/navigation_demo_thumbnail.png)]([https://www.youtube.com/watch?v=YOUR_VIDEO_ID](https://youtu.be/l4WzAzQvGUU?si=MSMofUL5z0Aud9e7))

**YouTube:** [Watch the complete navigation demonstration]([https://www.youtube.com/watch?v=YOUR_VIDEO_ID](https://youtu.be/l4WzAzQvGUU?si=MSMofUL5z0Aud9e7))

## Screenshots

### Starting

![AMCL localization in RViz](images/rviz_localization.png)

RViz showing the map, robot pose, laser scan, and AMCL localization.

### Nav2 Navigation

![Nav2 navigation](<img width="1850" height="1044" alt="Screenshot from 2026-09-24 16-48-43" src="https://github.com/user-attachments/assets/5cc71f61-de87-4c22-b45b-10bb25b70329" />)

RViz showing the generated navigation path and the robot moving toward the commanded goal.

### Successful Navigation

![Successful navigation](<img width="738" height="527" alt="Screenshot from 2026-09-24 16-54-44" src="https://github.com/user-attachments/assets/3b8ee12f-ab76-4ca2-87e4-0f3aa0eeb9ba" />)

Final robot position after successfully reaching the commanded navigation goal.
<img width="1850" height="1044" alt="Screenshot from 2026-09-24 16-48-54" src="https://github.com/user-attachments/assets/f6a5a10d-e36d-4eb9-99dd-c84037bbad34" />



---

# Contact

**Name:** Mukhtar Ahmed
**Contact:** +917305323969
**Email:** ahmedmukthor@gmail.com

