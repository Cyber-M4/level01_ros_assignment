# Level 1: ROS2 Navigation Assignment - Your Full Name

The objective of this assignment is to configure, debug, and implement an end-to-end autonomous navigation stack for a simulated differential-drive robot inside a custom Gazebo playground environment. Rather than utilizing monolithic bringup wrappers, the navigation pipeline is modularized into independently managed lifecycle nodes:

* **Manual Map Loading:** Configured via `nav2_map_server` and lifecycle management.
* **Robot Localization:** Implemented with `nav2_amcl` tuned for custom TF frames and simulated LiDAR constraints.
* **Autonomous Path Planning & Control:** Configured via `nav2_navfn_planner` (Global Planner), `dwb_core::DWBLocalPlanner` (Local Controller), `nav2_bt_navigator`, and layered costmaps.
* **Physics & Kinematics Calibration:** Corrected wheel friction and torque parameters to achieve 1:1 synchronization between Gazebo physical translation and Nav2 odometry.
* **Bug Fixing & Code Quality:** Identified and resolved multiple repository syntax, launch forwarding, XML, physics, and dependency bugs.

---

## 1. Package Structure

assignment_ws/
└── src/
    └── level01_ros_assignment/
        ├── testbed_description/         # Robot URDF, meshes, Gazebo plugins, and state publisher
        │   ├── meshes/
        │   ├── urdf/
        │   │   ├── testbed.urdf
        │   │   └── testbed.gazebo       # Calibrated diff-drive plugin, torque, and wheel friction
        │   └── launch/
        │       └── robot_description.launch.py
        │
        ├── testbed_gazebo/              # Simulation environment and world spawn files
        │   ├── models/
        │   ├── worlds/
        │   │   └── testbed_playground.world
        │   └── launch/
        │       ├── spawn_playground.launch.py
        │       └── spawn_testbed.launch.py
        │
        ├── testbed_bringup/             # High-level bringup launch files and maps
        │   ├── launch/
        │   │   └── testbed_full_bringup.launch.py
        │   └── maps/
        │       ├── testbed_world.yaml
        │       └── testbed_world.pgm
        │
        └── testbed_navigation/          # Custom Navigation Package (Implemented)
            ├── CMakeLists.txt           # Build instructions & share install rules
            ├── package.xml              # Declared build, exec, and launch dependencies
            ├── config/
            │   ├── amcl.yaml            # AMCL localization configuration (1.5m laser range, base_footprint)
            │   ├── nav2_params.yaml     # Planners, controllers, costmaps, and BT configs
            │   ├── testbed_world.yaml   # Map metadata (resolution: 0.05m, origin: [-10.2, -9.94, 0.0])
            │   └── testbed_world.pgm    # Map occupancy grid image (405 x 400 pixels)
            ├── launch/
            │   ├── map_server.launch.py   # Independent Map Server + Lifecycle Manager
            │   ├── localization.launch.py # Independent AMCL + Lifecycle Manager
            │   └── navigation.launch.py   # Nav2 Servers (Planner, Controller, BT, Behaviors)
            └── rviz/
                └── testbed_nav.rviz       # Pre-configured 20x20m grid Nav2 RViz layout

## 2. Bugs identified and resolved

During integration, multiple bugs and configuration flaws were identified and fixed across the repository:
#	Component / File	Issue Description	Root Cause	Solution Implemented
1	testbed_navigation/CMakeLists.txt	Build failure during colcon build.	Malformed CMake syntax: ament_package(.	Corrected to standard ament_package().
2	testbed_gazebo/launch/spawn_playground.launch.py	Gazebo launched with default empty world instead of playground.	world launch argument was declared but not forwarded to the included Gazebo launch action.	Added launch_arguments={'world': LaunchConfiguration('world')}.items().
3	testbed_bringup/maps/testbed_world.yaml	Map server crashed on startup (File not found).	Image path pointed to nonexistent wrong_path_testbed_world.pgm.	Corrected image path to image: testbed_world.pgm.
4	testbed_description/urdf/testbed.gazebo	URDF/SDF parsing warnings/errors.	Malformed XML tag with an extra trailing >: <...reference>>.	Cleaned tag to <initial_orientation_as_reference>false</initial_orientation_as_reference>.
5	testbed_description/package.xml	Description launch crashed on clean environments.	Missing joint_state_publisher and robot_state_publisher dependencies.	Added explicit <exec_depend> declarations in package.xml.
6	testbed_description/urdf/testbed.gazebo	Robot physically lagged in Gazebo while Nav2 reached the goal.	Drive wheels had low friction (0.2) and casters had high friction (0.2), causing severe wheel slip and odometry desync.	Boosted motor torque (50.0 Nm), set drive wheel friction to 1.0, and reduced caster friction to 0.001.
7	testbed_navigation/config/nav2_params.yaml	A circular ring of false obstacles appeared at 1.5m around the robot.	Default Nav2 configs assumed 30m LiDARs, so open-air 1.5m readings were marked as obstacles.	Set obstacle_max_range: 1.35, raytrace_max_range: 2.0, and inf_is_valid: true.


## 3. Technical Implementation

A. Map Server (map_server.launch.py)

    Node: nav2_map_server::MapServer

    Manages map topic /map with transient local durability.

    Lifecycle controlled via nav2_lifecycle_manager::LifecycleManager.

B. Localization (localization.launch.py & amcl.yaml)

    Node: nav2_amcl::AmclNode

    Filter particle count: min_particles: 500, max_particles: 2000.

    Ray-tracing limit aligned with physical simulation: laser_max_range: 1.5.

    Base frame: base_footprint.

    Auto-initial pose configured at

            
    (0.0,0.0,0.0)
    (0.0,0.0,0.0)

C. Navigation Stack (navigation.launch.py & nav2_params.yaml)

    Global Planner: nav2_navfn_planner/NavfnPlanner (Dijkstra / A* graph search).

    Local Controller: dwb_core::DWBLocalPlanner (Dynamic Window Approach trajectory rollouts evaluated with critics: RotateToGoal, BaseObstacle, PathAlign, GoalAlign, PathDist, GoalDist).

    Behavior Tree: nav2_bt_navigator managing recovery behaviors (spin, backup, wait, clear costmaps).

    Costmaps:

        Static Layer (loads /map).

        Obstacle Layer (2D LaserScan clearing/marking up to 1.35 m, min height 0.12 m).

        Inflation Layer (inflation_radius: 0.65, cost_scaling_factor: 2.5, robot_radius: 0.25).
        
## 4. Build and Instalitation

Clone the repository and install all required system dependencies:
code Bash

# 1. Create and navigate to workspace
mkdir -p ~/assignment_ws/src
cd ~/assignment_ws/src

# 2. Clone repository
git clone https://github.com/Cyber-M4/level01_ros_assignment.git

# 3. Install dependencies
cd ~/assignment_ws
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
sudo apt install ros-humble-joint-state-publisher ros-humble-nav2-bringup -y

# 4. Build workspace
colcon build --symlink-install
source install/setup.bash

## 5. Guide

Open dedicated terminals and execute the following commands sequentially:
Terminal 1: Launch Gazebo Simulation World
code Bash

source /usr/share/gazebo/setup.sh 2>/dev/null || source /usr/share/gazebo-11/setup.sh 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch testbed_gazebo spawn_playground.launch.py

Terminal 2: Robot Description & Spawning
code Bash

source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch testbed_description robot_description.launch.py

(In the same terminal or new tab, spawn the robot entity):
code Bash

ros2 launch testbed_gazebo spawn_testbed.launch.py

Terminal 3: Launch Map Server
code Bash

source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch testbed_navigation map_server.launch.py

Terminal 4: Launch AMCL Localization
code Bash

source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch testbed_navigation localization.launch.py

Terminal 5: Launch Nav2 Navigation Pipeline
code Bash

source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch testbed_navigation navigation.launch.py use_sim_time:=true

Terminal 6: Launch RViz Visualization
code Bash

source /opt/ros/humble/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true

## 6. Operating robot in RViz(Nav2)

    Initial Pose Estimation:

        Select 2D Pose Estimate in the RViz top toolbar.

        Click and drag the arrow at the robot's physical spawn position in the central room pointing forward.

        AMCL will process the laser scan, converge its particle cloud, and broadcast the map -> odom transform.

    Sending a Navigation Goal:

        Select Nav2 Goal (or 2D Goal Pose) in RViz.

        Click an unoccupied target location on the map (e.g., through a doorway into another room).

    Observation:

        The Global Planner generates a path (/plan) centered through doorways.

        The Local Controller computes velocity commands (/cmd_vel) avoiding obstacles with zero wheel slip.

        The robot autonomously travels to the goal, and bt_navigator outputs:
        code Text

        [INFO] [bt_navigator]: Navigation succeeded

## 7. Result
   <img width="1850" height="1044" alt="Screenshot from 2026-09-24 16-47-43" src="https://github.com/user-attachments/assets/45153101-831e-4836-bd08-72ff9830de45" />

   <img width="1850" height="1044" alt="Screenshot from 2026-09-24 16-48-54" src="https://github.com/user-attachments/assets/164e77af-b3c5-4134-a898-6ad02f2a0e4d" />

   <img width="738" height="527" alt="Screenshot from 2026-09-24 16-54-44" src="https://github.com/user-attachments/assets/cbde6d97-a9ed-48ce-8576-fb91263470b2" />

   <img width="1850" height="1044" alt="Screenshot from 2026-09-24 16-48-43" src="https://github.com/user-attachments/assets/76c4a72a-8977-4ea8-ac12-a864e43fe3a1" />



    Gazebo Simulation: Perspective view showing the robot spawned inside testbed_playground.world with obstacles.

    TF Tree Structure: Continuous transform pipeline validated from map down to lidar_link_1 via ros2 run tf2_tools view_frames.

    Map & Particle Cloud: RViz rendering showing laser alignment with map walls and particle convergence.

    Costmap Visualizations: Global and Local costmaps showing proper safety inflation zones without phantom obstacle rings.

    Path Execution: Real-time trajectory tracking and obstacle avoidance through narrow doorways.

    Mission Completion: Terminal output displaying [INFO] [bt_navigator]: Navigation succeeded.

## 8. Working Video

   https://www.youtube.com/@mukhtarahmed4147

## Contact Info 
 - Name: Mukhtar ahmed
 - Contact number: +917305323969
 - Email Address: ahmedmukthor@gmail.com
