# lab-01-ros2-robot-simulation: Program diffdrive_bot

This repository contains the complete implementation of **Lab 1: ROS 2 and Custom Robot Simulation** for the *Introduction to Robotics and Intelligent Systems* course.

The repository contains four ROS 2 packages designed to build and run self-contained within any ROS 2 Humble workspace.

---

## Repository Structure

```
lab-01-ros2-robot-simulation/
├── diffdrive_bot/            # Differential-drive robot description, sensors, world, and bridges
│   ├── config/bridge.yaml    # ros_gz_bridge configuration (lidar, cameras, IMU, odom, cmd_vel)
│   ├── launch/               # display.launch.py, gazebo.launch.py, control.launch.py
│   ├── urdf/                 # diffdrive_bot.urdf.xacro with depth camera & IMU additions
│   └── worlds/obstacles.sdf  # Gazebo simulation world with IMU system plugin
├── lab_interfaces/           # Base message, service, and action interfaces
│   ├── action/DriveDistance.action
│   ├── msg/ObstacleInfo.msg
│   └── srv/SetSpeed.srv
├── my_robot_interfaces/      # Custom interface definitions (ament_cmake)
│   ├── action/Rotate.action
│   ├── msg/RobotStatus.msg
│   └── srv/SetTurnSpeed.srv
├── my_robot_nodes/           # Custom Python nodes and launch files (ament_python)
│   ├── launch/square.launch.py
│   └── my_robot_nodes/
│       ├── turn_away.py      # Obstacle avoidance node with parameters and services
│       ├── rotate_server.py  # Action server rotating robot by yaw tracking
│       └── square_driver.py  # Action client executing 4-sided square trajectory
├── docs/                     # Verification screenshots
│   ├── rviz_depth_pointcloud.png
│   └── rqt_graph_square.png
└── README.md
```

---

## Requirements & Prerequisites

* **Operating System:** Ubuntu 22.04 LTS (native or WSL2)
* **ROS 2:** ROS 2 Humble (`ros-humble-desktop`)
* **Simulator:** Gazebo Harmonic with ROS 2 bridge (`ros-humble-ros-gzharmonic`)
* **Extra tools:** `ros-humble-xacro`, `ros-humble-teleop-twist-keyboard`, `ros-humble-rqt-graph`

---

## Build Instructions

To build these packages, ensure they are placed inside the `src` folder of a ROS 2 workspace (e.g. `~/ros2_ws/src`):

```bash
# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Navigate to your workspace root
cd ~/ros2_ws

# 3. Build with colcon
colcon build --symlink-install

# 4. Source your workspace overlay
source install/setup.bash
```

> **Note:** If you clone this repository alongside other course material containing packages with identical names (`diffdrive_bot` or `lab_interfaces`), place a `COLCON_IGNORE` file in the other directory so colcon builds only your `lab1_exercise` packages.

---

## Running and Verifying Each Step

### Step 1: Depth Camera and IMU Verification

1. **Verify URDF Frames and Transforms:**
   ```bash
   ros2 launch diffdrive_bot display.launch.py
   ```
   In a second terminal:
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   ros2 run tf2_ros tf2_echo base_link depth_camera_link
   ```
   *Expected result:* Translation `[-0.215, 0.0, 0.0]` and rotation around the z-axis of approximately `3.142` rad ($180^\circ$).

2. **Verify Gazebo Simulation, Sensor Streams & RViz:**
   ```bash
   ros2 launch diffdrive_bot gazebo.launch.py
   # Or in headless mode (higher simulation real-time factor):
   # ros2 launch diffdrive_bot gazebo.launch.py headless:=true
   ```
   In RViz:
   * Add an **Image** display listening to `/depth_camera/image`.
   * Add a **PointCloud2** display listening to `/depth_camera/points`.
   In a second terminal, verify the sensor streams in simulation time:
   ```bash
   # Use -s / --use-sim-time to measure frequency in simulation time:
   ros2 topic hz -s /imu
   ros2 topic hz -s /depth_camera/image
   ros2 topic echo /imu --once
   ```
   *Expected result:* In simulation time (`-s`), the IMU publishes at ~50 Hz and the depth camera publishes at ~10 Hz, with `linear_acceleration.z` $\approx 9.8\text{ m/s}^2$ while stationary. (Note: Measuring without `-s` yields wall-clock arrival rate $f_{\text{wall}} = f_{\text{sim}} \times \text{RTF}$. On systems where RTF is ~0.06, wall rate is ~2.9 Hz for IMU and ~0.56 Hz for depth camera).

---

### Steps 2 to 5: Obstacle Avoidance (`turn_away` Node)

The `turn_away` node steers the robot away from obstacles using `/obstacle` messages, publishes status to `/robot_status`, and provides service endpoints for state and speed control.

1. **Launch the robot controller simulation:**
   ```bash
   ros2 launch diffdrive_bot control.launch.py
   ```

2. **Start the `turn_away` node in a second terminal:**
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   ros2 run my_robot_nodes turn_away
   ```

3. **Verify initial disabled state (Step 5) and status topic (Step 4):**
   ```bash
   ros2 topic echo /robot_status --once
   ```
   *Expected output:* `state: "stopped"` and the current obstacle distance.

4. **Enable the node:**
   ```bash
   ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
   ```
   *Expected result:* Robot begins driving forward at `forward_speed` (default 0.2 m/s). When approaching an obstacle, it turns in place at `turn_speed` (default 0.5 rad/s) away from the obstacle.

5. **Test dynamic parameters (Step 3):**
   ```bash
   ros2 param set /turn_away forward_speed 0.4
   ```
   *Expected result:* Robot immediately drives forward faster without needing a node restart.

6. **Test speed service (Step 5):**
   ```bash
   # Valid speed (accepted)
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 1.0}"

   # Excessive speed (rejected by max_turn_speed parameter)
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 3.0}"
   ```

7. **Disable the node:**
   ```bash
   ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
   ```
   *Expected result:* Robot publishes a single zero velocity Twist, stops immediately, and state returns to `"stopped"`.

---

### Step 6: Rotate Action Server (`rotate_server`)

1. **Keep simulation running:**
   ```bash
   ros2 launch diffdrive_bot control.launch.py
   ```

2. **Start `rotate_server` in another terminal:**
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   ros2 run my_robot_nodes rotate_server
   ```

3. **Send a rotation goal with live feedback:**
   ```bash
   ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
   ```
   *Expected result:* The robot turns $90^\circ$ to the left. The terminal displays continuous feedback climbing to ~1.57 rad, terminates with status `SUCCEEDED`, and returns the final turned angle.

4. **Inspect the new robot heading:**
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```

---

### Step 7: Full Autonomous Square Routine (`square.launch.py`)

1. **Run the complete system in a single command:**
   ```bash
   ros2 launch my_robot_nodes square.launch.py side_length:=0.8
   ```
   *Note:* Do not run `gazebo.launch.py` or `control.launch.py` separately. `square.launch.py` already includes `control.launch.py` (which launches Gazebo, the robot description, and the ROS bridges) and automatically starts `rotate_server` and `square_driver`.

2. **Inspect the computation graph in a second terminal while the robot is driving:**
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   ros2 run rqt_graph rqt_graph
   ```
   *Important:* Run `rqt_graph` in the second terminal while the robot is driving. Once the square routine completes its four sides, `square_driver` prints its completion message and shuts down cleanly, so running `rqt_graph` after it stops will not show `/square_driver` in the active graph.

---

## Screenshots

### Step 1: RViz Depth Image & Point Cloud
![RViz Depth Image and Point Cloud](docs/rviz_depth_pointcloud.png)

### Step 7: rqt_graph Computation Graph
![rqt_graph while square.launch.py runs](docs/rqt_graph_square.png)

---

## Technical Reflection & Implementation Notes

### 1. Cumulative Yaw vs. Direct Yaw Subtraction
In `rotate_server`, the robot's heading is measured from odometry quaternions converted to planar yaw. Because planar orientation wraps around from $+\pi$ to $-\pi$ when turning backwards, subtracting `start_yaw` from `current_yaw` causes catastrophic discontinuous jumps (e.g. from $+3.14$ to $-3.14$).
To solve this, the server integrates small incremental changes between consecutive odometry updates:
$$\Delta\psi = \text{atan2}(\sin(\psi_k - \psi_{k-1}), \cos(\psi_k - \psi_{k-1}))$$
This delta is always small and continuous regardless of coordinate quadrant boundaries.

### 2. Multi-Threaded Execution for Action Servers
In ROS 2, an action server executing a blocking loop in its goal callback will starve subscribers on the default single-threaded executor. In both `drive_distance_server` and `rotate_server`, odometry subscribers must receive live callbacks while the action loop runs. This was resolved using `ReentrantCallbackGroup` and running `rclpy.spin(node, executor=MultiThreadedExecutor())`.

### 3. Separation of CMake and Python Packages
ROS 2 Humble requires message/service/action generation packages to use the `ament_cmake` build type. Therefore, interface definitions are cleanly isolated in `my_robot_interfaces` while node logic is implemented in the `ament_python` package `my_robot_nodes`.

---

## What Did Not Work and Why

### 1. Gazebo `gz_frame_id` Warning

- **What happened:** Gazebo printed warnings about the `<gz_frame_id>` tags in the IMU and depth camera sensor definitions.
- **Evidence:** The launch console reported that `gz_frame_id` was not defined in SDF and was copied under the sensor element:
  ```text
  [gazebo_gui-6] Warning [Utils.cc:132] [/sdf/model[@name="diffdrive_bot"]/link[@name="base_footprint"]/sensor[@name="depth_camera"]/gz_frame_id:<data-string>:L230]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
  [gazebo_gui-6] Warning [Utils.cc:132] [/sdf/model[@name="diffdrive_bot"]/link[@name="base_footprint"]/sensor[@name="imu"]/gz_frame_id:<data-string>:L237]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
  ```
- **Why I think it happened:** I believe the warning occurs because the tag is an extension that Gazebo's parser does not recognize as a standard SDF sensor element. Despite the warning, the sensors worked and their ROS messages contained the expected frame IDs (`imu_link` and `depth_camera_link`).

### 2. Low Sensor Rates

- **What happened:** Initially, `/imu` was not yet publishing. After the bridge started, `ros2 topic hz` showed lower rates than the configured sensor frequencies.
- **Evidence:**
  - Before the bridge completed initialization, inspecting `/imu` returned:
    ```text
    WARNING: topic [/imu] does not appear to be published yet
    ```
  - When measured without `-s` against wall-clock time, `ros2 topic hz` showed lower rates:
    - `/imu`: approximately 2.9 Hz (configured: 50 Hz)
    - `/depth_camera/image`: approximately 0.56 Hz (configured: 10 Hz)
    - `/depth_camera/points`: approximately 0.5 Hz (configured: 10 Hz)
  - When measured with simulation time using `ros2 topic hz -s`, the verified rates were:
    - `ros2 topic hz -s /imu`: approximately 50.0 Hz
    - `ros2 topic hz -s /depth_camera/image`: approximately 10.16 Hz
- **Why I think it happened:** I believe Gazebo was running slower than real time in WSL2 (simulation Real-Time Factor was approximately 0.055–0.06 due to software rendering of the depth camera's `R_FLOAT32` buffer on the CPU), so wall-clock message rates were lower than the configured simulation rates. Measuring with simulation time helped distinguish this from a sensor configuration problem.

### 3. Action Goal Rejection

- **What happened:** During startup of `square.launch.py`, `square_driver` logs showed that the initial `DriveDistance` action goal was rejected on side 1 before succeeding on a retry.
- **Evidence:**
  - In `~/.ros/log/python3_56343_1793603419265.log`, the rejection occurred immediately when starting side 1:
    ```text
    [INFO] [square_driver]: Starting square drive routine with side_length=1.00 m
    [INFO] [square_driver]: Waiting for action servers (/drive_distance and /rotate)...
    [INFO] [square_driver]: Action servers available. Beginning square sides...
    [INFO] [square_driver]: >>> Side 1/4: Driving forward 1.00 m <<<
    [WARN] [square_driver]: DriveDistance rejected (attempt 1/5). Retrying...
    [WARN] [square_driver]: DriveDistance rejected (attempt 2/5). Retrying...
    [WARN] [square_driver]: DriveDistance rejected (attempt 3/5). Retrying...
    [WARN] [square_driver]: DriveDistance rejected (attempt 4/5). Retrying...
    [INFO] [square_driver]: Drive succeeded: travelled 1.00 m
    ```
  - In `diffdrive_bot/scripts/drive_distance_server.py` (lines 64–67), the `on_goal` callback checks:
    ```python
    def on_goal(self, goal):
        if goal.distance <= 0.0 or self.position is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    ```
- **Why I think it happened:** When `square.launch.py` launched all nodes simultaneously, `square_driver` connected to `drive_distance_server` and requested the first drive goal before the `/odom` bridge published the first odometry message. Because `self.position` was still `None`, `drive_distance_server` rejected the goal. The retry loop in `square_driver.py` waited and retried every second; once odometry was received, the goal was accepted and the routine completed all four sides successfully.
