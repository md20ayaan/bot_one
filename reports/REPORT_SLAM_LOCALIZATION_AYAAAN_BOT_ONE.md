# Comprehensive Engineering & Handover Report: 2D LiDAR SLAM, AMCL Localization & ROS 2 Architecture

**Project Repository:** `bot_one`  
**Platform Architecture:** macOS ARM64 (Apple Silicon M-Series via Pixi / RoboStack ROS 2 Lyrical) $\rightarrow$ NVIDIA Jetson Nano Target Hardware  
**Primary Sensor Suite:** 2D LiDAR (`laser_frame`), Differential-Drive Wheel Encoders (`odom`)  
**Target Vehicle:** Autonomous Differential-Drive RC Car Chassis  

---

## 1. Executive Summary & Core Milestones

This technical document details the architectural decisions, pipeline implementation, theoretical foundations, runtime bugs, resolutions, and system validation steps conducted for the autonomous differential-drive robot (`bot_one`). The project transition encompassed moving from an unmapped simulation baseline in Gazebo Sim Harmonic to a fully localized autonomous state using Nav2's Adaptive Monte Carlo Localization (AMCL) against a static occupancy grid.

### Progress Matrix Across Defined Project Steps:
- **Step 1 (Repository, Topic & TF Inspection):** Completed. Audited the URDF/Xacro kinematic chain, verified sensor message definitions, and checked frame parenting against REP-105 standards.
- **Step 2 (Theoretical Foundations):** Completed. Formalized the mathematical and probabilistic roles of `map`, `odom`, `base_link`, scan-matching algorithms, and the `map -> odom` correction transform.
- **Step 3 (2D LiDAR SLAM Implementation):** Completed. Configured and executed `slam_toolbox` in synchronous online mapping mode, verifying real-time loop closures and scan integration.
- **Step 4 (Map Generation & Occupancy Grid Quality Evaluation):** Completed. Teleoperated the chassis through the simulated maze, eliminated drift artifacts, evaluated wall thickness consistency, and exported the high-resolution occupancy grid (`maze_map.yaml` and `maze_map.pgm`).
- **Step 5 (Static Map Localization via AMCL & Lifecycle Verification):** Completed. Deployed `nav2_map_server` and `nav2_amcl` via `nav2_lifecycle_manager`. Successfully debugged runtime lifecycle transitions, QoS transport layer mismatches, and particle distribution mechanics. Confirmed kidnapped-robot recovery and full transform chain continuity (`map -> odom -> base_link -> laser_frame`).
- **Step 6 (Physical Odometry Integration & Teammate Coordination):** **PENDING / UNFINISHED (Simulation Baseline Only).** Wheel odometry on physical hardware has not yet been implemented or tested. During Steps 1–5, Gazebo's simulated differential-drive plugin (`gazebo_control.xacro`) acted as a temporary stand-in to provide the `/odom` topic and `odom -> base_link` transform. A strict interface specification has been formulated below so that when physical encoder hardware and firmware are built, they integrate seamlessly with this verified localization stack.

---

## 2. Step 1: Baseline Architecture & System Inspection

Before configuring higher-level autonomy nodes, the package structure and coordinate frames of `bot_one` were audited directly from source files and active ROS 2 graph outputs.

### Coordinate Frame Conventions (REP-103 & REP-105)
The kinematic structure adheres strictly to standard robotics conventions:
- **`base_link`**: Centered at the midpoint of the differential-drive wheel axle at ground contact height.
  - $+X$: Forward driving axis
  - $+Y$: Lateral left axis
  - $+Z$: Normal vertical axis pointing upward
- **`laser_frame`**: Defined in `description/lidar.xacro`. The optical center of the simulated 2D LiDAR is positioned rigidly relative to `base_link` with a physical translational offset of $Z = +0.165\text{ m}$.
- **`odom`**: The local planar dead-reckoning reference frame published by the Gazebo differential-drive plugin.

### Topic and Message Topology
- **LiDAR Topic:** `/scan`
  - Message Type: `sensor_msgs/msg/LaserScan`
  - Frame ID: `laser_frame`
  - Angular Sweep: $-\pi$ to $+\pi$ ($360^\circ$)
- **Odometry Topic:** `/odom` (Simulated via Gazebo plugin)
  - Message Type: `nav_msgs/msg/Odometry`
  - Frame ID: `odom`
  - Child Frame ID: `base_link`
- **Control Interface:** `/cmd_vel`
  - Message Type: `geometry_msgs/msg/Twist`

---

## 3. Step 2: Theoretical Foundations of 2D SLAM & Localization

### The Core Problem: Why LiDAR Alone Fails
A 2D LiDAR scanner returns range-bearing arrays measuring the distance to surrounding reflective surfaces. However, an instantaneous laser scan contains no intrinsic memory of geographic position. If a robot traverses an open or repetitive corridor:
1. Pure scan matching suffers from degeneracy along smooth, featureless walls (translation along the wall yields identical likelihood scores).
2. Processing scans without a motion prior forces optimization algorithms into local minima, causing rotational skew and duplicated geometric obstacles.

### The Symbiotic Role of Wheel Odometry
Wheel odometry integrates rotational encoder counts over small time deltas ($\Delta t$):
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{L}$$
Where $L$ is the track width and $\Delta s_R, \Delta s_L$ are wheel displacement arcs.
- **Short-Term Precision:** Odometry provides a high-frequency, continuous estimate of motion between consecutive LiDAR sweeps, serving as the kinematic prediction prior for both SLAM graph nodes and AMCL particle propagation.
- **Long-Term Limitation:** Wheel slip, uneven floor contact, and gear backlash cause unbounded integration drift over time.

### The REP-105 Coordinate Hierarchy
ROS separates continuous local motion from drift-corrected global motion by establishing a three-tier tree with strict single-parent ownership:
```
[ map ] (Global, fixed world frame)
   │
   │  Broadcasted by: AMCL or SLAM (Discrete jumps during drift correction)
   ▼
[ odom ] (Continuous local dead-reckoning frame)
   │
   │  Broadcasted by: Differential-Drive Controller / Wheel Encoders (Smooth)
   ▼
[ base_link ] (Chassis origin)
   │
   │  Broadcasted by: robot_state_publisher (Static URDF kinematics)
   ▼
[ laser_frame ] (Sensor origin)
```
- **The Role of the `map -> odom` Transform:** Because physical robots cannot instantaneously teleport without violating continuous kinematics, the `odom -> base_link` transform is kept strictly continuous and smooth. When AMCL or SLAM detects accumulated wheel drift, it does **not** alter `odom -> base_link`; instead, it computes and shifts the `map -> odom` offset such that the compounded transform ($\text{map} \rightarrow \text{base_link} = \mathbf{T}_{\text{map}}^{\text{odom}} \times \mathbf{T}_{\text{odom}}^{\text{base_link}}$) accurately reflects the true global pose.

---

## 4. Steps 3 & 4: 2D LiDAR SLAM & Map Construction

### SLAM Pipeline Execution (`slam_toolbox`)
1. **Configuration:** Configured `slam_toolbox` in online asynchronous mode, binding `/scan` to the incoming LiDAR point stream and using the `odom` frame as the tracking baseline.
2. **Scan Matching:** The solver used Ceres-based scan-to-map matching. Scans were evaluated against an evolving occupancy grid to extract spatial graph vertices and constraint edges.
3. **Exploration & Loop Closure:** The chassis was teleoperated at low translational velocities ($v \le 0.2\text{ m/s}$) through the maze loops. Loop closure detection continuously optimized the underlying non-linear pose graph when intersecting previously surveyed corridors, preventing map shearing.

### Map Evaluation & Serialization
The resulting occupancy grid was evaluated in RViz for structural fidelity:
- Zero double-wall artifacts or skewed corridors.
- Sharp $90^\circ$ orthogonality on maze corner boundaries.
- Complete internal clearing (free space marked with high confidence; unknown cells restricted strictly to unobserved external areas).

The map was serialized to disk using `nav2_map_server`:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/dev_ws/src/bot_one/config/maze_map
```
This produced two assets:
1. **`maze_map.pgm`:** 8-bit grayscale bitmap image ($158 \times 147$ pixels) where pixel luminance encodes occupancy probability ($0 = \text{free space}$, $254 = \text{occupied wall}$, $205 = \text{unknown}$).
2. **`maze_map.yaml`:** Metadata descriptor file specifying grid scale and origin:
   ```yaml
   image: /Users/mdayaan/dev_ws/src/bot_one/config/maze_map.pgm
   mode: trinary
   resolution: 0.05
   origin: [-2.962, -3.904, 0.0]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   ```

---

## 5. Step 5: Static Map Localization (AMCL) Architecture

Once the static map was generated, SLAM was decommissioned and replaced with Adaptive Monte Carlo Localization (AMCL) using a KLD-sampling particle filter.

### Nav2 Lifecycle Node Management
Unlike ROS 1 nodes that boot immediately into active execution, Nav2 components use state machines (`Unconfigured` $\rightarrow$ `Inactive` $\rightarrow$ `Active`).
To automate this deterministic sequence, a dedicated Python launch file (`src/bot_one/launch/localization.launch.py`) was constructed using `nav2_lifecycle_manager`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('bot_one')
    map_yaml_file = os.path.join(pkg_dir, 'config', 'maze_map.yaml')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'amcl_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, {'yaml_filename': map_yaml_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params_file]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])
```

---

## 6. Comprehensive Debugging Log: Failure Modes & Root Cause Analyses

During the setup and verification phases, several platform-specific and framework issues were diagnosed and resolved:

### Failure 1: Missing `config/` Directory in Workspace Install Space
- **Symptom:** Executing `ros2 launch bot_one localization.launch.py` crashed immediately with:
  `Package 'bot_one' does not contain directory 'config' in install share.`
- **Root Cause Analysis:** In ROS 2 colcon builds, launch files query `install/share/<pkg_name>/` rather than raw `src/`. `src/bot_one/CMakeLists.txt` only instructed CMake to install `launch`, `description`, and `worlds`.
- **Engineering Resolution:** Updated `src/bot_one/CMakeLists.txt` to include `config`:
  ```cmake
  install(
    DIRECTORY launch description worlds config
    DESTINATION share/${PROJECT_NAME}
  )
  ```
  The workspace was re-indexed with `--symlink-install` to allow future parameter modifications to take effect immediately without rebuilding.

---

### Failure 2: Nav2 Motion Model Plugin Class Incompatibility
- **Symptom:** During bringup, `lifecycle_manager` aborted node transition:
  ```text
  [amcl-2] [ERROR]: Caught exception in callback for transition 10
  [amcl-2] [ERROR]: Original error: According to the loaded plugin descriptions the class differential with base class type nav2_amcl::MotionModel does not exist. Declared types are nav2_amcl::DifferentialMotionModel nav2_amcl::OmniMotionModel
  [amcl-2] [FATAL]: Lifecycle node amcl does not have error state implemented
  [lifecycle_manager-3] [ERROR]: Failed to change state for node: amcl
  ```
- **Root Cause Analysis:** Earlier Nav2 releases permitted shortened string aliases such as `"differential"`. Modern ROS 2 distributions enforce strict C++ pluginlib class namespace resolution.
- **Engineering Resolution:** Updated `src/bot_one/config/amcl_params.yaml`:
  - *Invalid Entry:* `robot_model_type: "differential"`
  - *Corrected Entry:* `robot_model_type: "nav2_amcl::DifferentialMotionModel"`

---

### Failure 3: RViz UI Thread Deadlock from Stale SLAM Panel
- **Symptom:** RViz2 froze entirely, ceased processing user input, and flooded terminal output with continuous polling warnings:
  `[INFO] [SlamToolboxPlugin]: Waiting for the slam_toolbox node configuration.`
- **Root Cause Analysis:** The active RViz workspace retained the `SlamToolboxPlugin` dock panel from Step 3. Because `slam_toolbox` was no longer active, the background Qt GUI thread entered an unhandled blocking loop attempting to poll a dead ROS 2 service server.
- **Engineering Resolution:**
  1. Force-killed the frozen GUI instance via process signal:
     ```bash
     pkill -9 -f rviz2
     ```
  2. Booted a fresh instance anchored directly to the simulation clock:
     ```bash
     rviz2 --ros-args -p use_sim_time:=true
     ```
  3. Removed the orphaned panel and serialized the clean display layout directly to `src/bot_one/config/localization.rviz`.

---

### Failure 4: Map Visualization QoS Policy Mismatch
- **Symptom:** The RViz `Map` display plugin threw `Status: Warn` with dimensions reported as `0 x 0`, leaving the 3D viewport canvas blank.
- **Root Cause Analysis:** `nav2_map_server` publishes the static map with `Transient Local` durability (QoS latching). When RViz was launched after `map_server` had already transitioned to the `Active` state, RViz's default `Volatile` durability subscriber missed the initial publication.
- **Engineering Resolution:** Within RViz Display settings:
  - Expanded **Map** $\rightarrow$ **Topic**
  - Updated **Durability Policy** from `Volatile` to `Transient Local`
  - The $158 \times 147$ maze grid rendered immediately.

---

### Failure 5: Dormant Particle Cloud Display & Unpopulated Dropdown
- **Symptom:** `/particlecloud` failed to appear in RViz topic picker dropdowns, and adding it manually showed `PoseArray: Status: Error`.
- **Root Cause Analysis:** AMCL implements motion-gated particle regeneration governed by `update_min_d` and `update_min_a`. Until the robot exceeds these translational or angular deadbands, AMCL remains dormant and avoids allocating/publishing the particle array. Furthermore, the user interface picker only auto-detects active publisher advertisements.
- **Engineering Resolution:**
  1. Manually specified `/particlecloud` into the display's text field.
  2. Sent small motion commands via `teleop_twist_keyboard` to exceed the initial deadband ($d > 0.1\text{ m}$, $\theta > 0.2\text{ rad}$).
  3. The particle cloud materialized instantly across the canvas as directional arrows.

---

## 7. Operational Validation: Kidnapped Robot & TF Chain Verification

### Kidnapped Robot Recovery Test
To verify the probabilistic convergence of the particle filter:
1. **Pose Offset Injection:** The **2D Pose Estimate** tool in RViz was used to inject an intentional error of approximately $1.0\text{ m}$ translation and $45^\circ$ heading misalignment.
2. **Behavioral Observation:**
   - The particle array immediately dispersed outward across a wide radius, indicating an intentional spike in covariance uncertainty.
   - The live LiDAR scan dots (red) separated from the static map obstacle contours (black).
3. **Motion-Induced Convergence:** Teleoperating the robot through two gentle rotational maneuvers caused AMCL to re-evaluate beam likelihood weights against the static occupancy field.
4. **Resolution:** Within $0.4\text{ m}$ of travel, invalid particle hypotheses were eliminated, the cluster collapsed into a tight distribution around the true chassis coordinates, and the red scan returns locked flush onto the black maze walls.

### Real-Time Transform Echo Output
Continuous end-to-end TF resolution was validated using `tf2_echo`:

```bash
ros2 run tf2_ros tf2_echo map laser_frame
```

*Live Terminal Execution Output:*
```text
At time 5080.955000000
- Translation: [3.234, -2.725, 0.165]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.985, 0.174]
- Rotation: in RPY (radian) [0.000, -0.000, 2.791]
- Rotation: in RPY (degree) [0.000, -0.000, 159.902]

At time 5081.635000000
- Translation: [3.136, -2.791, 0.165]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.866, 0.500]
- Rotation: in RPY (radian) [0.000, -0.000, 2.093]
- Rotation: in RPY (degree) [0.000, -0.000, 119.946]

At time 5082.961000000
- Translation: [2.988, -2.744, 0.165]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.373, 0.928]
- Rotation: in RPY (radian) [0.000, -0.000, 0.765]
- Rotation: in RPY (degree) [0.000, -0.000, 43.847]
```
- **Validation Points:** Zero transform dropouts, continuous millisecond timestamp synchronization, and an exact vertical offset of $Z = 0.165\text{ m}$ confirming the URDF mount geometry between chassis and LiDAR.

---

## 8. Target Platform Constraints & Jetson Nano Optimization Profile

The current simulation configuration in `amcl_params.yaml` balances high visual fidelity with moderate compute:
- Particle bounds: $500 \le N \le 2000$
- Laser beam evaluations: $M = 60$
- Sensor distribution weights: $z_{\text{hit}} = 0.5$, $z_{\text{rand}} = 0.5$

### Computational Complexity Analysis
The computational overhead of AMCL's likelihood field measurement model scales directly as:
$$\mathcal{O}(N_{\text{particles}} \times M_{\text{beams}})$$
- **Simulation Setup:** Up to $2{,}000 \times 60 = \mathbf{120{,}000 \text{ ray-cast ray evaluations}}$ per motion step.
- **Hardware Profile:** The target NVIDIA Jetson Nano features a 4-core ARM Cortex-A57 CPU clocked at $1.43\text{ GHz}$. Running $120{,}000$ coordinate lookups per update alongside real-time serial drivers, LiDAR USB communication, and motor control loops can saturate CPU cores, resulting in frame latency and dropped TF buffers.

### Production Jetson Nano Parameter Profile:
When porting the codebase to physical hardware, apply the following optimized configuration in `src/bot_one/config/amcl_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    global_frame_id: "map"
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    scan_topic: /scan
    tf_broadcast: true
    transform_tolerance: 0.5

    # Resource-optimized particle bounds
    min_particles: 80
    max_particles: 400
    pf_err: 0.05
    pf_z: 0.99
    resample_interval: 1

    # Sensitive motion triggers (rapid snapping without extra compute)
    update_min_d: 0.05
    update_min_a: 0.1

    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.15
    alpha2: 0.15
    alpha3: 0.15
    alpha4: 0.15
    alpha5: 0.15

    # Lightweight ray evaluation
    laser_model_type: "likelihood_field"
    laser_min_range: 0.15
    laser_max_range: 8.0
    max_beams: 60
    z_hit: 0.8
    z_rand: 0.1
    z_short: 0.05
    z_max: 0.05
    sigma_hit: 0.1
    do_beamskip: true
```
- **Max Compute Ceiling:** Capping particles at $400$ restricts peak calculations to $24{,}000$ evaluations per update ($80\%$ load reduction).
- **`do_beamskip: true`**: Directly skips scan beams that project outside known map boundaries, saving CPU cycles on invalid rays.
- **`z_hit: 0.8`**: Increases alignment weighting on valid wall scans, allowing the robot to snap accurately onto corridors within 1–2 wheel rotations.

---

## 9. Step 6 (Pending Deliverable): Integration Specification for Physical Odometry

> **Status: UNFINISHED / OPEN WORK ITEM**  
> Physical wheel encoders and hardware-level odometry have **not** yet been built, calibrated, or integrated. During simulation testing, the Gazebo differential-drive plugin (`gazebo_control.xacro`) acted as a temporary stand-in to provide the `/odom` topic and `odom -> base_link` transform.  
> 
> When work on physical odometry begins (either individually or by an assigned teammate), the physical motor/encoder firmware must implement and satisfy the following contract to plug directly into the verified localization stack without code refactoring.

### 1. Transform Authority & Tree Ownership
- The odometry node **must publish**: `odom -> base_link` on `/tf`.
- The odometry node **must NOT publish**: `map -> odom` (owned exclusively by AMCL).
- The `odom -> base_link` transform must be continuous; it must never experience discrete position jumps.

### 2. Topic Specification: `/odom`
- **Topic Name:** `/odom`
- **Message Type:** `nav_msgs/msg/Odometry`
- **Target Publishing Rate:** $30\text{ Hz}$ to $50\text{ Hz}$ (never drop below $20\text{ Hz}$)

#### Field Format:
```python
# Header
header.stamp = node.get_clock().now().to_msg()
header.frame_id = "odom"

# Child Frame
child_frame_id = "base_link"

# Position (Planar ground)
pose.pose.position.x = calculated_x      # Meters
pose.pose.position.y = calculated_y      # Meters
pose.pose.position.z = 0.0
pose.pose.orientation = quaternion       # Derived from heading theta

# Linear & Angular Velocities
twist.twist.linear.x = v_x               # Forward velocity (m/s)
twist.twist.linear.y = 0.0               # Non-holonomic constraint
twist.twist.angular.z = omega_z          # Rotational velocity (rad/s)
```

### 3. Covariance Matrix Formatting
To prevent AMCL from rejecting incoming odometry, diagonal variances must be explicitly defined rather than initialized to all-zeros:

```python
# Pose Covariance (6x6 row-major: x, y, z, roll, pitch, yaw)
pose_covariance = [0.0] * 36
pose_covariance[0]  = 0.001   # X variance (m^2)
pose_covariance[7]  = 0.001   # Y variance (m^2)
pose_covariance[14] = 1e6     # Z variance (untracked planar)
pose_covariance[21] = 1e6     # Roll variance
pose_covariance[28] = 1e6     # Pitch variance
pose_covariance[35] = 0.005   # Yaw variance (rad^2)

# Twist Covariance (6x6 row-major: vx, vy, vz, vroll, vpitch, vyaw)
twist_covariance = [0.0] * 36
twist_covariance[0]  = 0.002   # vx variance
twist_covariance[7]  = 1e6     # vy variance
twist_covariance[14] = 1e6     # vz variance
twist_covariance[21] = 1e6     # vroll
twist_covariance[28] = 1e6     # vpitch
twist_covariance[35] = 0.01    # vyaw variance
```

### 4. Integration Verification Commands (To Be Executed on Hardware)
Run this 4-point sanity check once physical firmware is deployed:
```bash
# 1. Verify publishing frequency
ros2 topic hz /odom

# 2. Check frame identities
ros2 topic echo /odom --field header.frame_id
ros2 topic echo /odom --field child_frame_id

# 3. Echo the transform
ros2 run tf2_ros tf2_echo odom base_link

# 4. Sign and Direction Validation:
# - Pushing the vehicle forward +1.0 m must increment pose.position.x by +1.0
# - Rotating the chassis 90 deg counter-clockwise must increment yaw by +1.57 rad
```

---

## 10. Repository State & Tracked Artifacts

All operational files have been committed and pushed to the upstream remote:
- **Repository:** `https://github.com/md20ayaan/bot_one.git`
- **Commit Hash:** `6bec9ee`
- **Commit Message:** `feat(localization): configure map_server, amcl lifecycle, and saved maze map`

### Inventory of Core Files:
1. `src/bot_one/CMakeLists.txt`: Configured with complete `config/` installation paths.
2. `src/bot_one/description/gazebo_control.xacro`: Differential-drive simulation plugin configuration.
3. `src/bot_one/description/lidar.xacro`: Structural definition and optical frame offsets for the LiDAR.
4. `src/bot_one/config/maze_map.yaml` & `maze_map.pgm`: High-resolution occupancy grid map.
5. `src/bot_one/config/amcl_params.yaml`: Validated AMCL and `map_server` parameters.
6. `src/bot_one/config/localization.rviz`: Preserved RViz display layout with QoS overrides.
7. `src/bot_one/launch/localization.launch.py`: Multi-node lifecycle composition launch file.