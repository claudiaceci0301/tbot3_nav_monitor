# tbot3_nav_monitor — Adaptive Navigation System for TurtleBot3

## Overview

This project implements an autonomous navigation stack for TurtleBot3 using ROS2 Humble and Nav2
inside a Docker-based simulation environment (Gazebo + Cartographer SLAM).

The system extends the standard TurtleBot3 navigation setup with three custom ROS2 nodes:

- **`adaptive_controller_node`** — runtime parameter adjustment based on collected metrics
- **`metric_collector_node`** — navigation metric collection and publication
- **`data_logger_node`** — CSV logging and alert system

The goal is to improve observability and robustness of autonomous navigation while maintaining 
full compatibility with the official TurtleBot3 simulation stack.

---

## Base System

| Component | Version / Image |
|-----------|----------------|
| Docker image | `merabro/turtlebot3-sim:latest` |
| Container name | `ros2_humble_new` |
| ROS2 | Humble |
| Navigation stack | Nav2 (planner, controller, BT navigator) |
| Simulation | Gazebo + TurtleBot3 world |
| SLAM | Cartographer |
| Visualization | Foxglove Studio |

---

## System Architecture

```
/goal_pose ──→ AdaptiveController ──→ Nav2 action server (navigate_to_pose)
                     ↑                        ↓
                     │              controller_server → /cmd_vel → robot
                     │                        ↑
              /navigation_metrics      behavior_server (recovery)
                     ↑
              MetricCollector
                     ↑
              /odom, /scan, /cmd_vel
```

### Topic and Service Map

| Topic / Service | Direction | Description |
|----------------|-----------|-------------|
| `/goal_pose` | in | Goal sent from Foxglove |
| `/navigation_metrics` | out (MetricCollector) / in (AdaptiveController, DataLogger) | Custom metric message |
| `/nav2_goal_status` | out (AdaptiveController) / in (MetricCollector) | Nav2 result (SUCCEEDED/ABORTED/CANCELED) |
| `/metric_collector/reset` | service | Resets MetricCollector state on new goal |
| `/odom` | in | Robot odometry |
| `/scan` | in | LiDAR data |
| `/cmd_vel` | in | Last commanded velocity |

---

## Node Details

### `adaptive_controller_node` (LifecycleNode)

Subscribes to `/navigation_metrics` and dynamically adjusts Nav2 parameters via `AsyncParametersClient`.

**Adaptive conditions:**

| Condition | Trigger | Action |
|-----------|---------|--------|
| Cond 1 | `recovery_count > threshold` | Reduce `max_vel_x`, `max_vel_theta`; increase `inflation_radius` |
| Cond 2 | `mean_accuracy < threshold` (window) | Relax `xy_goal_tolerance`, `yaw_goal_tolerance` |
| Cond 3 | `mean_obstacle_proximity < threshold` (window) | Increase `inflation_radius` |
| Cond 4 | `efficiency < threshold` or obstacle too close | Increase `inflation_radius`; reduce `GridBased.tolerance` |

Conditions 2–4 require a complete evaluation window (`window_size` samples near the goal). 
Condition 1 is evaluated on every message.

**Rate limiting** — each of the three Nav2 clients (controller, costmap, planner) has an independent cooldown interval 
and diff check to avoid unnecessary parameter updates:

- Controller: 0.5s (2 Hz) - faster
- Costmap: 1.0s (1 Hz) - middle
- Planner: 2.0s (0.5 Hz) - slower

**Goal flow:**
1. `/goal_pose` received → `send_nav2_goal()` → Nav2 action server
2. `MetricCollector` reset requested via `/metric_collector/reset`
3. `result_callback` publishes `/nav2_goal_status`
4. On goal reached: all parameters restored to normal, window reset and cmd_vel set to 0 trough the topic '/cmd_vel_filtered'

### `metric_collector_node` (LifecycleNode)

Timer-driven at `publish_rate` Hz. Collects and publishes `NavigationMetrics` messages.

**Collected fields:**

| Field | Description |
|-------|-------------|
| `distance_traveled` | Cumulative odometry distance [m] |
| `battery_consumption` | Simulated battery drain [%] |
| `min_obstacle_distance` | Minimum LiDAR distance (3 sectors: left, center, right) [m] |
| `recovery_count` | Nav2 recovery events detected |
| `goal_reached` | True when Nav2 SUCCEEDED or geometry fallback triggered |
| `distance_to_goal` | Euclidean distance to current target [m] |
| `optimal_path` | Straight-line start→goal distance [m] |
| `nav2_state` | Last Nav2 result code (0=UNKNOWN, 1=SUCCEEDED, 2=ABORTED, 3=CANCELED) |

**Recovery detection** — a recovery event is counted when:
- `collision_detection()` returns true (obstacle within `obstacle_distance_tolerance`)
- `cmd_vel` was non-zero but odometry position is unchanged (robot stuck)

**Geometry fallback** — if `AdaptiveController` is down, `goal_reached_` is set when 
the robot enters the geometric tolerance independently of Nav2.

**Goal reached guard** — once `goal_reached_` is true, the control loop publishes 
a final complete message, the final cmd velocity set to 0
and stops updating counters (no spurious recovery increments after goal).

### `data_logger_node` (rclcpp::Node)

Subscribes to `/navigation_metrics` and `/odom`. Not a lifecycle node — always active.

**CSV columns:** `real_time_ms`, `sim_time_ms`, `timestamp_ms`, `publish_time_ms`, `distance_traveled`, 
`battery_consumption`, `min_obstacle_distance`, `goal_reached`, `distance_to_goal`

**Alert system:**

| Alert | Condition |
|-------|-----------|
| Battery ERROR | consumption > 95% |
| Battery WARN | consumption > 85% |
| Stagnant distance WARN | `distance_to_goal` not decreasing for N ticks |
| Latency WARN | publish→receive delay > 100ms |
| Latency ERROR | publish→receive delay > 500ms |
| Odom rate ERROR | rate < 5.0 Hz |
| Odom rate WARN | rate < 10.0 Hz |

The odom rate check skips the first measurement window to avoid false positives during node startup.

---

## Custom Message — `NavigationMetrics`

```
std_msgs/Header header
float64 distance_traveled
float64 battery_consumption
float64 min_obstacle_distance
int32   recovery_count
bool    goal_reached
float64 current_x
float64 current_y
float64 current_theta
float64 distance_to_goal
float64 distance_tolerance
float64 angle_tolerance
float64 obstacle_distance_tolerance
float64 optimal_path
uint8   nav2_state
```

---

## Nav2 Configuration

All Nav2 parameters are consolidated in `config/params.yaml`, which fully replaces the Nav2 default configuration. 
Key tuned values:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `controller_frequency` | 15 Hz | Reduced from 20 Hz to lower CPU load in simulation |
| `xy_goal_tolerance` | 0.35 m | Larger tolerance for stable goal reaching in simulation |
| `yaw_goal_tolerance` | 0.35 m | Idem |
| `inflation_radius` | 0.5 m | Safe clearance from obstacles |
| `movement_time_allowance` | 20 s | Allows time for recovery behaviors |
| `required_movement_radius` | 0.2 m | Progress checker sensitivity |

The adaptive controller modifies these parameters at runtime.
On goal reached, all parameters are restored to the values defined in `params.yaml` (aka. 'normal_[namevariable]').

---

## Docker Setup

```bash
# Clean up previous container and image
docker stop ros2_humble_new
docker rm ros2_humble_new
docker rmi ros2_humble_dev

# Build and start
docker-compose build
docker-compose up -d

# Verify
docker logs ros2_humble_new

# Enter container
docker exec -it ros2_humble_new bash
```

```Foxglove

1. When docker-compose.yaml is started, the Foxglove Bridge is launched using the following command sequence. 
These commands configure the bridge by setting the listening port and loading the parameter configuration file:

command:
  - bash
  - -c
  - >
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 run foxglove_bridge foxglove_bridge
      --ros-args
      -p port:=8765
      -p address:=0.0.0.0
      --params-file /workspace/config/foxglove_bridge_params.yaml

The bridge is configured to listen on port 8765 and accept connections from any network interface (0.0.0.0).
Additional parameters are loaded from the file foxglove_bridge_params.yaml.

```

### Kill stale processes (if needed)

```bash
pkill -f gz
pkill -f ros2
pkill -f cartographer
pkill -f nav2
pkill gzclient
pkill gzserver
ros2 daemon stop
ros2 daemon start
```

---

## Launch

```bash
# Inside the container
colcon build
source install/setup.bash
bash /workspace/start.sh
```

`start.sh` starts in sequence:
1. Gazebo simulation (`turtlebot3_world`)
2. Cartographer SLAM
3. Nav2 bringup with `config/params.yaml`
4. `tbot3_nav_monitor` launch file (all three custom nodes)

After launch start.sh is done, open Foxglove Studio and connect to `ws://localhost:8765`. 
Send a goal using the Nav2 goal tool — the robot will navigate and all metrics will appear on `/navigation_metrics`.
---

## Foxglove Dashboard

Recommended panels:

- **3D** — robot, map, costmap, path
- **Raw Messages** — `/navigation_metrics` (all fields including `nav2_state`)
- **Plot** — `recovery_count`, `distance_to_goal`, `min_obstacle_distance`
- **Plot** — `battery_consumption`, `distance_traveled`, `optimal_path`
- **Log** — `/rosout` filtered by node name

In tbot3_nav_monitor/config the .json file has been saved with the default foxglove setup.

---

## Known Limitations

- **Control loop frequency** — the controller misses its target rate in complex scenarios due to CPU constraints 
                          in the Docker simulation environment. This is a hardware/simulation limit.
- **Goal oscillation** — after `Reached the goal!`, Nav2 may execute spin/wait recovery behaviors before publishing SUCCEEDED.
                      During this window the robot oscillates slightly around the goal hence the distance_to_goal oscillates too.
- **Geometry fallback precision** — the geometry fallback uses the odom frame, which drifts from the map frame over time. 
                                So the fallback may trigger slightly off from the true goal position.
- **Costmap resizing** — the global costmap resizes frequently during SLAM as the map grows. 
                    This contributes to planner loop misses.

---

## Design Decisions

- **Separation of concerns** — metric collection (`MetricCollector`) and adaptation logic (`AdaptiveController`) 
                          are separate lifecycle nodes with independent state machines.
- **Single decision point** — `compute_desired_params()` evaluates all conditions and returns a single `Nav2Params` snapshot; 
                        `apply_params()` pushes it to Nav2 in three independent async calls with per-client rate limiting and diff checking.
- **Window-based statistics** — accuracy and obstacle proximity use a sliding window of samples near the goal,
                          preventing premature adaptation before sufficient data is collected.
- **`navigation_active_` guard** — the `AdaptiveController` ignores metrics when no goal is active,
                              preventing spurious parameter changes between navigations.
                              Indeed the navigation_active_ is set to true when send_nav2_goal is called.
- **`std::atomic` for Nav2 state** — used for variables shared between action callbacks and the metrics callback.
                              Since the state is represented by a simple value, std::atomic provides thread-safe 
                              read/write access without the use of a mutex.
- **`std::lock_guard<std::mutex> lock(state_mutex_)`** — provides thread-safe access to shared resources by 
                                                    automatically managing mutex locking and unlocking within a scope.
---

## Experimental Results

The robot successfully navigates to the given goal within the configured distance tolerance. 
The adaptive controller correctly detects high recovery counts and reduces velocity in response. 
The data logger produces CSV files suitable for offline analysis.

Remaining challenges:
- Planner sensitivity to frequent costmap resizing during active SLAM
- Control loop frequency misses in CPU-constrained simulation environments

---

## Demo

*[Insert 5-minute demo video link here]*
