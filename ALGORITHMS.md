# F1/10th Racing Stack - Algorithm Analysis

This document provides a comprehensive overview of the key algorithms used in the NTU Autonomous Racing Team's F1/10th race stack, which was deployed at the IEEE IV 2024 competition in Jeju, Korea.

## Table of Contents
1. [System Architecture Overview](#system-architecture-overview)
2. [Path Following - Pure Pursuit](#path-following---pure-pursuit)
3. [Local Planning - Gap Finder (Disparity Extender++)](#local-planning---gap-finder-disparity-extender)
4. [Reactive Navigation - Wall Follow](#reactive-navigation---wall-follow)
5. [Safety Systems](#safety-systems)
   - [Time to Collision (TTC)](#time-to-collision-ttc)
   - [Automatic Emergency Braking (AEB)](#automatic-emergency-braking-aeb)
   - [Safety Node](#safety-node)
6. [Control Systems - PID Controller](#control-systems---pid-controller)
7. [Trajectory Optimization](#trajectory-optimization)
8. [References](#references)

---

## System Architecture Overview

The race stack follows a modular ROS2-based architecture with the following main components:

```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSORS                                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │
│  │   LiDAR     │    │   IMU/Odom  │    │  Joystick   │          │
│  │   /scan     │    │   /odom     │    │   /joy      │          │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘          │
└─────────┼──────────────────┼──────────────────┼─────────────────┘
          │                  │                  │
          ▼                  ▼                  │
┌─────────────────────────────────────────────────────────────────┐
│                     PERCEPTION & SAFETY                          │
│  ┌─────────────────────┐    ┌─────────────────────┐             │
│  │   Time to Collision │    │   Particle Filter   │             │
│  │       (TTC)         │    │   (Localization)    │             │
│  └──────────┬──────────┘    └──────────┬──────────┘             │
│             │                          │                         │
│             ▼                          │                         │
│  ┌─────────────────────┐               │                         │
│  │        AEB          │               │                         │
│  │ (Emergency Braking) │               │                         │
│  └──────────┬──────────┘               │                         │
└─────────────┼──────────────────────────┼────────────────────────┘
              │                          │
              ▼                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                        PLANNING                                  │
│  ┌─────────────────────┐    ┌─────────────────────┐             │
│  │    Gap Finder       │    │    Pure Pursuit     │             │
│  │  (Local Planner)    │    │  (Global Planner)   │             │
│  └──────────┬──────────┘    └──────────┬──────────┘             │
│             │                          │                         │
│             └────────────┬─────────────┘                         │
│                          ▼                                       │
│                  ┌──────────────┐                                │
│                  │    Demux     │                                │
│                  │  (Selector)  │                                │
│                  └──────┬───────┘                                │
└─────────────────────────┼───────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                      SAFETY LAYER                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    Safety Node                            │   │
│  │  • Dead-man switch (Joystick R1)                          │   │
│  │  • AEB multiplexing                                       │   │
│  │  • Drive command filtering                                │   │
│  └──────────────────────────────┬───────────────────────────┘   │
└─────────────────────────────────┼───────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ACTUATORS                                  │
│                   ┌─────────────┐                                │
│                   │   /drive    │                                │
│                   │   (VESC)    │                                │
│                   └─────────────┘                                │
└─────────────────────────────────────────────────────────────────┘
```

---

## Path Following - Pure Pursuit

### Overview
Pure Pursuit is a geometric path tracking algorithm that computes steering commands to follow a predefined waypoint path. It's used as the **global planner** for time-trial scenarios where an optimal raceline is pre-computed.

### Mathematical Foundation

The algorithm uses a **lookahead distance** to find a goal point on the path, then calculates the steering angle needed to reach that point using the bicycle model kinematics.

**Key Equations:**

1. **Curvature Calculation:**
   ```
   κ = 2 * y_goal / L²
   ```
   Where:
   - `κ` = curvature (1/radius)
   - `y_goal` = lateral offset of the goal point in vehicle frame
   - `L` = lookahead distance

2. **Steering Angle:**
   ```
   δ = Kp * κ
   ```
   Where:
   - `δ` = steering angle
   - `Kp` = proportional gain (default: 1.0)

3. **Steering Limits:**
   ```
   δ ∈ [-0.36, 0.36] radians  (Pure Pursuit specific - conservative for stability)
   ```
   Note: Different components may use different steering limits based on their use case. The hardware maximum is 0.4 rad.

### Algorithm Steps

1. **Load Waypoints**: Read pre-recorded waypoints from CSV file (x, y, target_speed)
2. **Get Current Pose**: Subscribe to odometry to get current position and heading
3. **Find Goal Point**: 
   - Iterate through waypoints
   - Check if waypoint is in front of the car (angle < 90°)
   - Find waypoint within lookahead distance (±0.5m tolerance)
4. **Transform to Vehicle Frame**: Use TF2 to transform goal point from map frame to vehicle frame
5. **Calculate Steering**: Apply pure pursuit formula
6. **Publish Command**: Send Ackermann drive message with steering angle and waypoint speed

### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookahead_distance` | 2.0m | Distance to look ahead for goal point |
| `Kp` | 1.0 | Steering proportional gain |
| `steering_max` | 0.36 rad | Maximum steering angle |

### Source Files
- `f1tenth_ws/src/pure_pursuit/scripts/pure_pursuit.py`
- `f1tenth_ws/src/pure_pursuit/scripts/pure_pursuit_python.py`

---

## Local Planning - Gap Finder (Disparity Extender++)

### Overview
The Gap Finder is a reactive local planner based on the **Disparity Extender** algorithm, enhanced with additional signal processing for stability. It finds the deepest gap in LiDAR scans to navigate around obstacles in real-time.

### Algorithm Components

#### 1. Disparity Extender
Detects sudden depth changes (disparities) in LiDAR scans that indicate obstacle edges, then extends a safety bubble to prevent the car from steering into narrow gaps.

**Disparity Detection:**
```
if |range[i] - range[i-1]| > disparity_threshold:
    mark as disparity point
```

#### 2. Safety Bubble
Applies a protective bubble around detected obstacles and the nearest point to prevent collisions.

**Bubble Size Calculation:**
```
radius_count = safety_bubble_diameter / (angle_increment * range) / 2
```

#### 3. Lookahead Distance Limiting
Clips all ranges beyond the lookahead distance to prioritize closer obstacles:
```
ranges[ranges > lookahead_distance] = lookahead_distance
```

#### 4. Field of View Limiting
Restricts the search space to a cone in front of the car (default: π radians = 180°)

#### 5. Center Priority Mask
Applies a subtle mask favoring center-pointing goals for straight-line driving:
```
mask = linspace(0.999, 1.0, half_scan_size)  # Applied symmetrically
```

#### 6. Speed Controller
Uses front clearance and physics-based calculations:

**Maximum Cornering Speed (Bicycle Model):**
```
v_max = √(g * μ * L / |tan(δ)|)
```
Where:
- `g` = 9.81 m/s² (gravity)
- `μ` = coefficient of friction (default: 0.71)
- `L` = wheelbase (0.324m)
- `δ` = steering angle

**Speed Scaling:**
```
speed = (front_clearance / lookahead_distance) * speed_max
```

### Configuration Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `disparity_bubble_diameter` | 0.4m | Safety bubble at disparity points |
| `safety_bubble_diameter` | 0.4m | Safety bubble at nearest point |
| `front_bubble_diameter` | 0.33m | Bubble for front clearance averaging |
| `view_angle` | π rad | Field of view cone |
| `disparity_threshold` | 0.6m | Threshold for edge detection |
| `lookahead_distance` | 10m | Maximum range to consider |
| `coefficient_of_friction` | 0.71 | Tire-road friction coefficient |
| `wheel_base` | 0.324m | Distance between axles |
| `speed_min` | 1.0 m/s | Minimum speed |
| `speed_max` | 10.0 m/s | Maximum speed |
| `steering_max` | 0.5 rad | Maximum steering angle (Gap Finder specific) |

> **Note on Steering Limits**: The Gap Finder uses a higher steering limit (0.5 rad) compared to Pure Pursuit (0.36 rad) because reactive obstacle avoidance may require more aggressive maneuvers. The hardware maximum is 0.4 rad, so the Gap Finder's value is clamped in practice.

### Feature Toggles
- `do_limit_lookahead`: Clip ranges at lookahead distance
- `do_mark_disparity`: Apply disparity extender
- `do_mark_minimum`: Add bubble at nearest point
- `do_limit_field_of_view`: Restrict search cone
- `do_prioritise_center`: Apply center bias mask
- `do_bin_speed`: Discretize speed into bins
- `do_speed_low_pass_filter`: Smooth speed commands
- `do_steering_low_pass_filter`: Smooth steering commands

### Source Files
- `f1tenth_ws/src/gap_finder/scripts/gap_finder_base.py` (Primary implementation)
- `f1tenth_ws/src/gap_finder/gap_finder/pid.py`
- `f1tenth_ws/src/_deprecated/fast_gap_finder/src/gap_finder_node.cpp` (Legacy C++ implementation - archived)

---

## Reactive Navigation - Wall Follow

### Overview
Wall Follow implements a PID-based controller to maintain a constant distance from a wall, useful for track segments with clear boundaries.

### Mathematical Foundation

**Distance Calculation (Two-Angle Method):**

Using two LiDAR measurements at angles `a` (40°) and `b` (90°):

```
α = -arctan((a * cos(θ) - b) / (a * sin(θ)))
actual_distance = b * cos(α)
```

Where:
- `θ` = angle difference between measurements (50°)
- `α` = angle between wall and car heading
- `actual_distance` = perpendicular distance to wall

**Lookahead Error:**
```
error_1 = (desired_distance - actual_distance) + L * sin(α)
```
Where `L` is the velocity-dependent lookahead distance.

### PID Controller
```
steering = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
```

**Default Gains:**
| Gain | Value |
|------|-------|
| Kp | 0.40 |
| Ki | 0.030 |
| Kd | 0.002 |

### Speed Control
Uses the bicycle model for maximum cornering speed:
```
speed = min(5.0, √((g * μ * L) / |tan(δ)|))
```

### Source Files
- `f1tenth_ws/src/wall_follow/scripts/wall_follow.py`

---

## Safety Systems

### Time to Collision (TTC)

#### Overview
Calculates the time until potential collision based on LiDAR ranges and vehicle velocity.

#### Algorithm
```
TTC = min(range_i / (v * cos(θ_i)))
```
Where:
- `range_i` = distance measurement at angle `θ_i`
- `v` = longitudinal velocity
- `θ_i` = angle of measurement from forward direction

Only positive TTC values (approaching obstacles) are considered.

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `view_angle` | 1 rad | Angular range for TTC calculation |

#### Source Files
- `f1tenth_ws/src/time_to_collision/scripts/ttc_base.py`

---

### Automatic Emergency Braking (AEB)

#### Overview
Monitors TTC and triggers emergency stop when collision is imminent.

#### Algorithm
```python
if TTC <= threshold:
    AEB = 1  # Collision imminent
else:
    AEB = 0  # Safe
```

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `threshold` | 0.1s | TTC threshold for emergency braking |
| `pub_rate` | 10 Hz | Publishing rate |

#### Source Files
- `f1tenth_ws/src/automatic_emergency_braking/scripts/aeb_base.py`
- `f1tenth_ws/src/automatic_emergency_braking/scripts/aeb_ackermann.py`

---

### Safety Node

#### Overview
Acts as a drive command multiplexer, applying safety gains based on:
1. **Dead-man switch** (Joystick R1 button)
2. **AEB status**

#### Logic
```python
output_speed = input_speed * joy_gain * aeb_gain
```

| Condition | joy_gain | aeb_gain |
|-----------|----------|----------|
| R1 pressed | 1.0 | (current) |
| R1 released | 0.0 | reset to 1.0 |
| AEB triggered | (current) | 0.0 |

#### Source Files
- `f1tenth_ws/src/safety_node/scripts/safety_node.py`

---

## Control Systems - PID Controller

### Overview
A reusable PID controller class used throughout the stack.

### Implementation
```python
class PID:
    def update(self, feedback):
        error = set_point - feedback
        integral += Ki * error  # With anti-windup
        derivative = error - prev_error
        return Kp * error + integral + Kd * derivative + bias
```

### Features
- Configurable gains (Kp, Ki, Kd)
- Optional bias term
- Anti-windup with max_integral limit

### Source Files
- `f1tenth_ws/src/gap_finder/gap_finder/pid.py`
- `f1tenth_ws/src/gap_finder/scripts/pid.py`

---

## Trajectory Optimization

### Overview
Offline trajectory optimization for generating optimal racelines. Supports multiple objectives:
1. **Shortest Path**
2. **Minimum Curvature**
3. **Minimum Time**
4. **Minimum Time with Powertrain Consideration**

### Minimum Curvature Optimization
Uses quadratic programming to find a smooth raceline that minimizes curvature while respecting track boundaries.

### Minimum Time Optimization
Considers vehicle dynamics including:
- Friction limits (g-g diagram)
- Acceleration capabilities
- Tire-road friction coefficients
- Optional powertrain thermal limits

### Output Format
The optimized trajectory includes:
- `s_m`: Curvilinear distance
- `x_m, y_m`: Coordinates
- `psi_rad`: Heading
- `kappa_radpm`: Curvature
- `vx_mps`: Target velocity
- `ax_mps2`: Target acceleration

### Source Files
- `utils/trajectory_optimization/main_globaltraj_f110.py`
- `utils/trajectory_optimization/opt_mintime_traj/`
- `utils/trajectory_optimization/helper_funcs_glob/`

---

## References

### Academic Papers
1. **Disparity Extender Algorithm**: Otterness, N. (2019). "The Disparity Extender Algorithm and Its Successor" - [Blog Post](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html)
2. **Follow the Gap Method**: Sezer, V. & Gokasan, M. (2012). "A novel obstacle avoidance algorithm: follow the gap method", *Robotics and Autonomous Systems*, 60(9), pp. 1123-1134. DOI: [10.1016/j.robot.2012.05.021](https://doi.org/10.1016/j.robot.2012.05.021)
3. **Pure Pursuit**: Coulter, R.C. (1992). "Implementation of the Pure Pursuit Path Tracking Algorithm", *CMU Robotics Institute Technical Report*, CMU-RI-TR-92-01
4. **Minimum Curvature Planning**: Heilmeier, A. et al. (2019). "Minimum Curvature Trajectory Planning and Control for an Autonomous Racecar", *Vehicle System Dynamics*, 58(10), pp. 1497-1527. DOI: [10.1080/00423114.2019.1631455](https://doi.org/10.1080/00423114.2019.1631455)
5. **Time-Optimal Planning**: Christ, F. et al. (2019). "Time-Optimal Trajectory Planning for a Race Car Considering Variable Tire-Road Friction Coefficients", *Vehicle System Dynamics*, 59(4), pp. 588-612. DOI: [10.1080/00423114.2019.1704804](https://doi.org/10.1080/00423114.2019.1704804)

### External Resources
- [F1TENTH Lab Exercises](https://github.com/f1tenth/f1tenth_labs_openrepo)
- [Unifying F1TENTH Autonomous Racing: Survey, Methods and Benchmarks](https://arxiv.org/pdf/2402.18558)
- [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros)

### Hardware Specifications
| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheelbase | 0.32m | Distance between front and rear axles |
| Track Width | 0.26m | Distance between left and right wheels |
| Length | 0.55m | Overall vehicle length |
| Width | 0.30m | Overall vehicle width |
| Max Speed | 11.1 m/s | Motor limit |
| Max Steering | 0.4 rad | Hardware steering limit |

> **Note**: Some algorithm implementations use slightly different wheelbase values (e.g., 0.324m in Gap Finder) for fine-tuning. The official chassis measurement is 0.32m.

---

## Summary

The F1/10th race stack employs a hierarchical approach:

1. **Global Planning** (Pure Pursuit): Follows pre-computed optimal racelines for time trials
2. **Local Planning** (Gap Finder): Reactive obstacle avoidance for head-to-head racing
3. **Safety Layer**: Multi-level protection including TTC monitoring, AEB, and dead-man switch
4. **Control**: PID-based controllers with physics-based speed limits

The modular ROS2 architecture allows flexible combinations of these components depending on race mode (time trial vs. head-to-head).
